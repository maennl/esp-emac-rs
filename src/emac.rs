// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! Native ESP32 EMAC driver.
//!
//! Owns the DMA engine and drives the bring-up sequence directly via
//! the local register helper modules in `crate::regs::*` and
//! [`crate::reset::ResetController`]. No `ph-esp32-mac` dependency.

use embedded_hal::delay::DelayNs;

use crate::regs::dma as dma_regs;
use crate::regs::ext as ext_regs;
use crate::regs::gpio as gpio_matrix;
use crate::regs::mac as mac_regs;
use crate::reset::ResetController;

use crate::config::{ClkGpio, EmacConfig, RmiiClockConfig};
use crate::dma::engine::DmaEngine;
use crate::error::EmacError;
use crate::interrupt::InterruptStatus;
use crate::regs::dma::{bus_mode, operation};
use crate::regs::mac::{config, frame_filter};

const TX_FIFO_FLUSH_TIMEOUT_US: u32 = 100_000;

// =============================================================================
// Link parameters and driver state
// =============================================================================

// Re-export the link-parameter enums from the trait crate so a PHY
// driver's `LinkState` lands directly into `set_speed` / `set_duplex`
// without the call-site `.into()` boilerplate that was needed when
// these were duplicate local types. Keeping the types in one place
// (eth_mdio_phy) also means a future minor-release variant addition
// (`Speed::_1000M`) propagates through both ends of the stack with
// a single bump.
//
// Gated by the `mdio-phy` feature because that feature is what pulls
// `eth_mdio_phy` in as a dependency. Users without the feature can
// still drop down to `crate::regs::mac::set_speed_100mbps` /
// `set_duplex_full` directly — see the module-level docs.
#[cfg(feature = "mdio-phy")]
pub use eth_mdio_phy::{Duplex, Speed};

/// EMAC driver state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmacState {
    /// Not yet initialized.
    Uninitialized,
    /// `init()` succeeded but DMA/MAC are not running.
    Initialized,
    /// `start()` succeeded — DMA active, can transmit/receive.
    Running,
}

// =============================================================================
// EMAC driver
// =============================================================================

/// ESP32 EMAC driver with statically allocated DMA buffers.
///
/// The DMA descriptor chain is self-referential, so the driver MUST be
/// placed in its final memory location BEFORE [`init`](Self::init) is
/// called.
pub struct Emac<const RX: usize = 10, const TX: usize = 10, const BUF: usize = 1600> {
    dma: DmaEngine<RX, TX, BUF>,
    config: EmacConfig,
    state: EmacState,
    mac_address: [u8; 6],
    /// Last applied speed setting; idempotency guard for [`Self::set_speed`].
    /// `None` means speed has not been applied yet (default before first
    /// `set_speed` call). PHY-link state polling calls `set_speed` every
    /// cycle — without the guard each call would issue a redundant MMIO
    /// write regardless of whether the parameters actually changed.
    ///
    /// **Caveat:** this cache can become stale if a consumer bypasses the
    /// API by calling [`crate::regs::mac::set_speed_100mbps`] /
    /// [`crate::regs::mac::set_duplex_full`] directly. Mixing the high-level
    /// [`Self::set_speed`] / [`Self::set_duplex`] with those low-level
    /// helpers is **not supported** — the idempotency guard will early-return
    /// on a stale cached value and leave the hardware configured against
    /// expectations. Pick one API and stick with it.
    #[cfg(feature = "mdio-phy")]
    current_speed: Option<Speed>,
    /// Last applied duplex setting; analogous to `current_speed` (same
    /// caveat about mixing with `regs::mac::set_duplex_full`).
    #[cfg(feature = "mdio-phy")]
    current_duplex: Option<Duplex>,
}

impl<const RX: usize, const TX: usize, const BUF: usize> Emac<RX, TX, BUF> {
    /// Create a new (uninitialized) driver.
    pub const fn new(config: EmacConfig) -> Self {
        Self {
            dma: DmaEngine::new(),
            config,
            state: EmacState::Uninitialized,
            mac_address: [0; 6],
            #[cfg(feature = "mdio-phy")]
            current_speed: None,
            #[cfg(feature = "mdio-phy")]
            current_duplex: None,
        }
    }

    // ── State accessors ────────────────────────────────────────────────────

    #[inline(always)]
    pub fn state(&self) -> EmacState {
        self.state
    }

    #[inline(always)]
    pub fn mac_address(&self) -> [u8; 6] {
        self.mac_address
    }

    #[inline(always)]
    pub fn config(&self) -> &EmacConfig {
        &self.config
    }

    /// Total static memory used by this EMAC instance.
    pub const fn memory_usage() -> usize {
        DmaEngine::<RX, TX, BUF>::memory_usage()
    }

    // ── Configuration ──────────────────────────────────────────────────────

    /// Set the MAC address.
    ///
    /// If the driver has been initialized, the hardware filter registers
    /// are updated immediately.
    pub fn set_mac_address(&mut self, mac: [u8; 6]) {
        self.mac_address = mac;
        if self.state != EmacState::Uninitialized {
            crate::regs::mac::set_mac_address(&mac);
        }
    }

    /// Apply the link speed reported by the PHY.
    ///
    /// The ESP32 EMAC peripheral physically supports only 10 Mbps and
    /// 100 Mbps. `Speed` is `#[non_exhaustive]` in the trait crate, so
    /// future variants (e.g. a hypothetical `_1000M`) compile but
    /// have no register encoding here. They are clamped to 100 Mbps —
    /// the highest mode the EMAC actually supports — and a warning is
    /// emitted under the `defmt` feature so the discrepancy is
    /// visible at runtime.
    ///
    /// Available only with the `mdio-phy` feature, which is also what
    /// pulls in the [`Speed`] type from `eth_mdio_phy`. Without the
    /// feature, drop down to [`crate::regs::mac::set_speed_100mbps`].
    #[cfg(feature = "mdio-phy")]
    pub fn set_speed(&mut self, speed: Speed) {
        if self.state == EmacState::Uninitialized {
            return;
        }
        // Idempotency guard — avoid redundant MMIO writes when PHY reports
        // unchanged link parameters. Without the guard, link-state polling
        // every 500 ms would issue a write at every poll regardless of change.
        if self.current_speed == Some(speed) {
            return;
        }
        let is_100 = match speed {
            Speed::_10M => false,
            Speed::_100M => true,
            _ => {
                #[cfg(feature = "defmt")]
                defmt::warn!(
                    "esp-emac: unsupported Speed variant, clamping to 100 Mbps \
                     (ESP32 EMAC is 10/100 only)"
                );
                true
            }
        };
        mac_regs::set_speed_100mbps(is_100);
        self.current_speed = Some(speed);
    }

    /// Apply the duplex mode reported by the PHY.
    ///
    /// `Duplex` is `#[non_exhaustive]` in the trait crate. ESP32 EMAC
    /// has only the two MII-canonical modes (Half/Full); any future
    /// variant is clamped to Full (the more permissive default) with
    /// a `defmt::warn!` so the unexpected input doesn't pass silently.
    ///
    /// Available only with the `mdio-phy` feature, which is also what
    /// pulls in the [`Duplex`] type from `eth_mdio_phy`. Without the
    /// feature, drop down to [`crate::regs::mac::set_duplex_full`].
    #[cfg(feature = "mdio-phy")]
    pub fn set_duplex(&mut self, duplex: Duplex) {
        if self.state == EmacState::Uninitialized {
            return;
        }
        // Idempotency guard — see [`Self::set_speed`].
        if self.current_duplex == Some(duplex) {
            return;
        }
        let is_full = match duplex {
            Duplex::Half => false,
            Duplex::Full => true,
            _ => {
                #[cfg(feature = "defmt")]
                defmt::warn!(
                    "esp-emac: unsupported Duplex variant, clamping to Full \
                     (ESP32 EMAC supports Half/Full only)"
                );
                true
            }
        };
        mac_regs::set_duplex_full(is_full);
        self.current_duplex = Some(duplex);
    }

    // ── Initialization ─────────────────────────────────────────────────────

    /// Initialize the EMAC peripheral.
    ///
    /// Sequence (mirrors the canonical ESP32 GMAC bring-up):
    /// 1. APLL 50 MHz programming — only when MCU is the RMII clock master
    ///    (`RmiiClockConfig::InternalApll` / `InternalApllClkOutGpio0`);
    ///    skipped for `External`.
    /// 2. RMII reference-clock pad routing (GPIO0 input for External,
    ///    GPIO16/17 output for InternalApll, or GPIO0 output via the SoC
    ///    clock-output mux for InternalApllClkOutGpio0).
    /// 3. SMI + RMII data-pin routing.
    /// 4. DPORT EMAC peripheral clock enable.
    /// 5. PHY interface mode (RMII) + clock source select.
    /// 6. EMAC extension clocks + RAM power-up.
    /// 7. DMA software reset.
    /// 8. MAC config defaults (PS/FES/DM/ACS/JD/WD).
    /// 9. DMA bus mode + operation mode defaults.
    /// 10. DMA descriptor chains and base-address registers.
    /// 11. MAC address program.
    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), EmacError> {
        if self.state != EmacState::Uninitialized {
            return Err(EmacError::AlreadyInitialized);
        }

        // 0. Validate user-configurable pins before touching any
        //    registers, so a bad `EmacConfig::pins` is rejected loudly
        //    rather than silently writing to unintended MMIO.
        if !gpio_matrix::is_valid_smi_pin(self.config.pins.mdc)
            || !gpio_matrix::is_valid_smi_pin(self.config.pins.mdio)
        {
            return Err(EmacError::InvalidConfig);
        }

        // RMII reference-clock pad direction on ESP32 is fixed by the
        // IO_MUX function:
        //
        // - GPIO0  function 5 = `EMAC_TX_CLK`         — INPUT only
        // - GPIO16 function 5 = `EMAC_CLK_OUT`        — OUTPUT only
        // - GPIO17 function 5 = `EMAC_CLK_OUT_180`    — OUTPUT only
        //
        // External clock therefore requires GPIO0 (the only input pad);
        // internal APLL output requires GPIO16 or GPIO17. Any other
        // combination is hardware-impossible — reject it before we
        // start writing IO_MUX bits.
        match self.config.clock {
            RmiiClockConfig::External { gpio } if !matches!(gpio, ClkGpio::Gpio0) => {
                return Err(EmacError::InvalidConfig);
            }
            RmiiClockConfig::InternalApll {
                gpio: ClkGpio::Gpio0,
                ..
            } => {
                return Err(EmacError::InvalidConfig);
            }
            _ => {}
        }

        // 1. APLL — programmed only when the MCU is the RMII clock
        //    master. SDM coefficients are picked from the configured
        //    on-board crystal (`xtal`) so the same code lands on
        //    50 MHz on 26/32/40 MHz boards alike. APLL is independent
        //    of the EMAC peripheral clock (only writes RTC analog +
        //    ROM I2C on the always-on APB), so order here doesn't
        //    matter. Skipped entirely for `External`.
        //
        //    `InternalApllClkOutGpio0` also needs the APLL running (its
        //    GPIO0 pad routing goes through the separate SoC clock-output
        //    mux, not the EMAC-dedicated pins, but the APLL programming
        //    step is identical).
        match self.config.clock {
            RmiiClockConfig::InternalApll { xtal, .. }
            | RmiiClockConfig::InternalApllClkOutGpio0 { xtal } => {
                crate::clock::configure_apll_50mhz(xtal);
            }
            RmiiClockConfig::External { .. } => {}
        }

        // 2. Route the RMII reference-clock pad: input on GPIO0 for
        //    `External`, output on GPIO16/17 for `InternalApll` (EMAC
        //    IO_MUX function 5), or output on GPIO0 for
        //    `InternalApllClkOutGpio0` (SoC clock-output mux, since
        //    GPIO0's EMAC function 5 is input-only).
        match self.config.clock {
            RmiiClockConfig::External { gpio } => crate::clock::configure_emac_clk_in(gpio),
            RmiiClockConfig::InternalApll { gpio, .. } => {
                crate::clock::configure_emac_clk_out(gpio)
            }
            RmiiClockConfig::InternalApllClkOutGpio0 { .. } => {
                crate::clock::configure_apll_clkout_gpio0()
            }
        }

        // 3. Configure SMI pins (MDC/MDIO from `EmacConfig::pins`) and
        //    RMII data pins (fixed function 5 — not configurable).
        gpio_matrix::configure_smi_pins(self.config.pins.mdc, self.config.pins.mdio);
        gpio_matrix::configure_rmii_pins();

        // 4. Enable EMAC peripheral clock through DPORT.
        ext_regs::enable_peripheral_clock();

        // 5. PHY interface — RMII with the appropriate clock source.
        ext_regs::set_rmii_mode();
        match self.config.clock {
            RmiiClockConfig::External { .. } => ext_regs::set_rmii_clock_external(),
            RmiiClockConfig::InternalApll { .. } | RmiiClockConfig::InternalApllClkOutGpio0 { .. } => {
                ext_regs::set_rmii_clock_internal()
            }
        }

        // 6. EMAC extension clocks + RAM power.
        ext_regs::enable_clocks();
        ext_regs::power_up_ram();

        // 7. Software reset of the DMA controller. `ResetController::new`
        //    uses the canonical `crate::reset::SOFT_RESET_TIMEOUT_MS`
        //    default — single source of truth for the reset window.
        //    `ResetError::Timeout` converts to `EmacError::DmaResetTimeout`
        //    via the `From` impl, so callers can distinguish DMA-stuck
        //    from MDIO timeouts.
        let mut reset_ctrl = ResetController::new(BorrowedDelay(delay));
        reset_ctrl.soft_reset()?;

        // 8. MAC configuration defaults: 100 Mbps full duplex, port select,
        //    auto pad/CRC strip, jabber + watchdog disabled.
        //
        //    CHECKSUM_OFFLOAD (IPC, bit 10) is **disabled**. The ESP32 GMAC
        //    checksum engine on at least rev v3.1 silicon is unreliable for
        //    both directions:
        //      * TX insertion (TDES0.CIC=0b11) produced bad checksums and
        //        broke TCP throughput after the first MSS-sized segment
        //        (see `TxDescriptor::prepare`).
        //      * RX verification (IPC=1) symmetrically marks valid frames
        //        as having checksum errors, which the DMA then drops at
        //        DMAOPERATION.DT=0 before they reach the CPU. This was
        //        observed as iperf2 downlink throughput collapsing to
        //        0 Mbps while uplink still trickled data through.
        //    Both sides therefore stay off and smoltcp computes / verifies
        //    IPv4/TCP/UDP/ICMP checksums in software (see
        //    `Driver::capabilities` advertising `ChecksumCapabilities::default()`).
        let mac_cfg = config::PORT_SELECT
            | config::SPEED_100
            | config::DUPLEX_FULL
            | config::AUTO_PAD_CRC_STRIP
            | config::JABBER_DISABLE
            | config::WATCHDOG_DISABLE;
        mac_regs::set_config(mac_cfg);

        // Frame filter: pass all multicast (broadcast accepted by default).
        mac_regs::set_frame_filter(frame_filter::PASS_ALL_MULTICAST);
        mac_regs::set_hash_table(0);

        // 9. DMA bus mode and operation mode.
        //
        // ATDS = enhanced 8-word descriptor layout (32 bytes per
        // descriptor). Our `dma::descriptor::{TxDescriptor,
        // RxDescriptor}` are now 8 words to match.
        let pbl = 32u32;
        let bus = bus_mode::FIXED_BURST
            | bus_mode::AAL
            | bus_mode::USP
            | bus_mode::ATDS
            | ((pbl << bus_mode::PBL_SHIFT) & bus_mode::PBL_MASK);
        dma_regs::set_bus_mode(bus);
        dma_regs::set_operation_mode(operation::TSF | operation::RSF);
        dma_regs::disable_all_interrupts();
        dma_regs::clear_all_interrupts();

        // 10. Descriptor chains. Returns physical base addresses suitable for
        //     DMARXBASEADDR / DMATXBASEADDR.
        let (rx_base, tx_base) = self.dma.init();
        dma_regs::set_rx_desc_list_addr(rx_base);
        dma_regs::set_tx_desc_list_addr(tx_base);

        // 11. Programme the MAC address into ADDR0H / ADDR0L (with AE bit).
        //     The internal filter latch on this Synopsys GMAC fires on the
        //     LOW write — `regs::mac::set_mac_address` writes HIGH first to
        //     keep the AE bit, then LOW to trigger the latch.
        crate::regs::mac::set_mac_address(&self.mac_address);

        self.state = EmacState::Initialized;
        Ok(())
    }

    /// Start TX/RX (DMA + MAC).
    pub fn start(&mut self) -> Result<(), EmacError> {
        match self.state {
            EmacState::Initialized => {}
            EmacState::Running => return Ok(()),
            EmacState::Uninitialized => return Err(EmacError::NotInitialized),
        }

        // Reset descriptor ownership in case of a previous run, then
        // re-program `DMARXBASEADDR` / `DMATXBASEADDR` from the base
        // addresses the engine returns. `dma.reset()` rebuilds chains
        // and zeroes the software `current_index`; the hardware DMA
        // pointer wherever it last was (middle of the ring after a
        // `stop()`/`start()` cycle, or unset on the very first start)
        // must be put back on the chain head, otherwise software and
        // hardware will walk different descriptors and RX wedges.
        let (rx_base, tx_base) = self.dma.reset();
        dma_regs::set_rx_desc_list_addr(rx_base);
        dma_regs::set_tx_desc_list_addr(tx_base);

        dma_regs::clear_all_interrupts();
        dma_regs::enable_default_interrupts();

        // Enable MAC TX, then DMA TX, DMA RX, then MAC RX (matches the
        // ordering from the ESP32 reference manual / IDF EMAC driver).
        let cfg = mac_regs::config();
        mac_regs::set_config(cfg | config::TX_ENABLE);

        dma_regs::start_tx();
        dma_regs::start_rx();

        let cfg = mac_regs::config();
        mac_regs::set_config(cfg | config::RX_ENABLE);

        // Issue an RX poll demand so the DMA does not stay in Suspended
        // state if all descriptors were already CPU-owned.
        dma_regs::rx_poll_demand();

        self.state = EmacState::Running;
        Ok(())
    }

    /// Stop TX/RX.
    ///
    /// Polls the TX-FIFO flush bit (`FTF`) for up to
    /// `TX_FIFO_FLUSH_TIMEOUT_US` microseconds, sleeping `delay` between
    /// polls so the DMA actually has time to drain. The rest of the
    /// teardown (MAC RX/TX disable, DMA RX stop, interrupt-status
    /// clear, state transition to `Initialized`) runs unconditionally
    /// — even on flush timeout the driver winds up in `Initialized`
    /// and is safe to re-`start()`.
    ///
    /// Returns:
    /// - `Ok(())` on a clean teardown (FTF self-cleared in time).
    /// - `Err(EmacError::TxFlushTimeout)` when the FTF poll exhausted
    ///   `TX_FIFO_FLUSH_TIMEOUT_US`. Teardown still completed — at
    ///   least one in-flight TX frame may have been truncated on the
    ///   wire. `state` is `Initialized` either way, so a follow-up
    ///   `start()` is the recoverable path. There is no in-crate
    ///   "full re-init" — [`Emac::init`] is one-shot — so a terminal
    ///   recovery means a peripheral or SoC reset from the
    ///   application layer.
    /// - `Err(EmacError::NotInitialized)` if called from `Uninitialized`.
    ///
    /// Idempotent on an already-stopped driver: calling `stop` while
    /// in `Initialized` returns `Ok(())` without touching hardware.
    pub fn stop(&mut self, delay: &mut impl DelayNs) -> Result<(), EmacError> {
        match self.state {
            EmacState::Running => {} // proceed with the tear-down below
            EmacState::Initialized => return Ok(()),
            EmacState::Uninitialized => return Err(EmacError::NotInitialized),
        }

        // Stop DMA TX, wait for in-flight data to drain (best effort).
        dma_regs::stop_tx();

        // Flush TX FIFO and wait for the bit to self-clear.
        dma_regs::flush_tx_fifo();
        const POLL_STEP_US: u32 = 10;
        let mut waited_us = 0u32;
        let mut flush_timed_out = true;
        while waited_us < TX_FIFO_FLUSH_TIMEOUT_US {
            if (dma_regs::operation_mode() & operation::FTF) == 0 {
                flush_timed_out = false;
                break;
            }
            delay.delay_us(POLL_STEP_US);
            waited_us += POLL_STEP_US;
        }

        // Disable MAC TX and RX, then DMA RX.
        let cfg = mac_regs::config();
        mac_regs::set_config(cfg & !(config::TX_ENABLE | config::RX_ENABLE));

        dma_regs::stop_rx();
        dma_regs::disable_all_interrupts();
        // Acknowledge any W1C bits that latched in DMASTATUS while the
        // engine was running, so a future `start()` doesn't observe
        // stale flags through `last_dmastat` / `interrupt_status` and
        // a re-enable from outside the driver doesn't fire spuriously.
        dma_regs::clear_all_interrupts();

        self.state = EmacState::Initialized;

        if flush_timed_out {
            Err(EmacError::TxFlushTimeout)
        } else {
            Ok(())
        }
    }

    // ── Frame I/O ─────────────────────────────────────────────────────────

    /// Transmit a frame.
    ///
    /// Does not block on descriptor availability — caller must check
    /// [`can_transmit`](Self::can_transmit) (or [`tx_ready`](Self::tx_ready)
    /// for single-descriptor frames) before calling, or be ready to handle
    /// `EmacError::NoDescriptorsAvailable` / `EmacError::DescriptorBusy`
    /// when the TX ring is full, and `EmacError::FrameTooLarge` when the
    /// payload exceeds the ring's combined capacity.
    pub fn transmit(&mut self, data: &[u8]) -> Result<usize, EmacError> {
        if self.state != EmacState::Running {
            return Err(EmacError::NotInitialized);
        }
        let n = self.dma.transmit(data)?;
        // Kick TX DMA out of suspended state if we just refilled descriptors.
        dma_regs::tx_poll_demand();
        Ok(n)
    }

    /// Receive a frame, if any.
    ///
    /// Issues an RX poll-demand whenever a descriptor was potentially
    /// recycled by `DmaEngine::receive` — that includes the success
    /// path (`Ok(Some(_))`) **and** the error paths (`FrameError`,
    /// `BufferTooSmall`, …) where the engine still hands the descriptor
    /// back to the DMA. Only `Ok(None)` skips the kick, since nothing
    /// in the ring changed. Without this, an errored frame on a
    /// suspended ring would leave RX wedged with the `RU` bit asserted
    /// until the next *successful* receive — exactly the kind of
    /// post-error hang we hit in the field.
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<Option<usize>, EmacError> {
        if self.state != EmacState::Running {
            return Err(EmacError::NotInitialized);
        }
        let result = self.dma.receive(buffer);
        if !matches!(result, Ok(None)) {
            dma_regs::rx_poll_demand();
        }
        result
    }

    /// Whether a received frame is currently waiting in the ring.
    #[inline(always)]
    pub fn rx_available(&self) -> bool {
        self.dma.rx_available()
    }

    /// Whether the TX ring has room for a frame of `len` bytes.
    #[inline(always)]
    pub fn can_transmit(&self, len: usize) -> bool {
        self.dma.can_transmit(len)
    }

    /// Whether at least one TX descriptor is available for the next frame.
    #[inline(always)]
    pub fn tx_ready(&self) -> bool {
        self.dma.tx_available() > 0
    }

    // ── Interrupt helpers ──────────────────────────────────────────────────

    /// Bind an interrupt handler to the EMAC peripheral and enable the
    /// interrupt at the chip level.
    #[cfg(feature = "esp-hal")]
    pub fn bind_interrupt(&mut self, handler: esp_hal::interrupt::InterruptHandler) {
        use esp_hal::peripherals::Interrupt;

        for core in esp_hal::system::Cpu::other() {
            esp_hal::interrupt::disable(core, Interrupt::ETH_MAC);
        }
        esp_hal::interrupt::bind_handler(Interrupt::ETH_MAC, handler);
        esp_hal::interrupt::enable(Interrupt::ETH_MAC, handler.priority());
    }

    /// Disable the EMAC interrupt at the chip level.
    #[cfg(feature = "esp-hal")]
    pub fn disable_interrupt(&mut self) {
        use esp_hal::peripherals::Interrupt;
        esp_hal::interrupt::disable(esp_hal::system::Cpu::current(), Interrupt::ETH_MAC);
    }

    /// Read and parse the DMA status register.
    pub fn interrupt_status(&self) -> InterruptStatus {
        // SAFETY: read from a known-valid memory-mapped register.
        let raw = unsafe {
            core::ptr::read_volatile(
                (crate::regs::dma::BASE + crate::regs::dma::DMASTATUS) as *const u32,
            )
        };
        InterruptStatus::from_raw(raw)
    }

    /// Clear DMA status flags via write-1-to-clear.
    ///
    /// Writes the raw register snapshot back into `DMASTATUS`,
    /// masked against [`crate::regs::dma::status::ALL_INTERRUPTS`] so
    /// only the documented W1C interrupt bits are touched. The
    /// non-W1C fields in `DMASTATUS` — `RS`/`TS` (process state),
    /// `EB` (error bits), `MMC`/`PMT`/`TTI` — are read-only and
    /// silently ignored by the hardware on write, but masking them
    /// keeps the contract explicit: every bit we send is something
    /// we mean to acknowledge.
    ///
    /// Pass the raw snapshot you previously read so every W1C bit
    /// (including ones not modeled in [`InterruptStatus`] such as
    /// `ERI` / `ETI` / `RWT`) is acknowledged in a single write.
    pub fn clear_interrupts_raw(&self, raw: u32) {
        // SAFETY: write to a known-valid memory-mapped register.
        unsafe {
            core::ptr::write_volatile(
                (crate::regs::dma::BASE + crate::regs::dma::DMASTATUS) as *mut u32,
                raw & crate::regs::dma::status::ALL_INTERRUPTS,
            );
        }
    }

    /// Convenience: handle the ISR — read status, clear all flags
    /// (via the raw snapshot, so unrepresented W1C bits are also
    /// acknowledged), return the parsed copy.
    pub fn handle_interrupt(&self) -> InterruptStatus {
        // SAFETY: read from a known-valid memory-mapped register.
        let raw = unsafe {
            core::ptr::read_volatile(
                (crate::regs::dma::BASE + crate::regs::dma::DMASTATUS) as *const u32,
            )
        };
        self.clear_interrupts_raw(raw);
        InterruptStatus::from_raw(raw)
    }
}

// `Default for Emac` is intentionally not implemented. The clock and pin
// configuration is hardware-specific and silently picking one (e.g.
// internal APLL on GPIO17) would mis-drive any board that expects an
// external PHY-driven clock or that routes MDC/MDIO to non-default
// GPIOs. Callers must construct an explicit `EmacConfig` — see the
// crate-level docs and `RmiiClockConfig` for the available modes.

// ── Default ring sizings ──────────────────────────────────────────────
//
// Single source of truth for the const generics that parameterize the
// `EmacDefault` / `EmacSmall` aliases on the MAC side and the matching
// `EmacDefaultDriver` / `EmacSmallDriver` aliases in `embassy.rs`. Keep
// the driver aliases pulled from these constants — retuning a value
// here updates both alias families together.

/// RX descriptor ring size for [`EmacDefault`].
pub const DEFAULT_RX: usize = 10;
/// TX descriptor ring size for [`EmacDefault`].
pub const DEFAULT_TX: usize = 10;
/// Per-buffer length (bytes) for [`EmacDefault`] / [`EmacSmall`].
pub const DEFAULT_BUF: usize = 1600;

/// RX descriptor ring size for [`EmacSmall`].
pub const SMALL_RX: usize = 4;
/// TX descriptor ring size for [`EmacSmall`].
pub const SMALL_TX: usize = 4;

/// Convenience alias: [`DEFAULT_RX`] RX / [`DEFAULT_TX`] TX /
/// [`DEFAULT_BUF`]-byte buffers (10/10/1600).
pub type EmacDefault = Emac<DEFAULT_RX, DEFAULT_TX, DEFAULT_BUF>;

/// Convenience alias: [`SMALL_RX`] RX / [`SMALL_TX`] TX /
/// [`DEFAULT_BUF`]-byte buffers (4/4/1600).
pub type EmacSmall = Emac<SMALL_RX, SMALL_TX, DEFAULT_BUF>;

/// RX descriptor ring size for [`EmacBench`].
pub const BENCH_RX: usize = 32;
/// TX descriptor ring size for [`EmacBench`].
pub const BENCH_TX: usize = 16;

/// Deeper-ring EMAC configuration for high-pps or bursty workloads.
///
/// **Ring sizing:** 32 RX × 16 TX × [`DEFAULT_BUF`]-byte buffers.
/// **Memory footprint:** ≈ 76.5 KiB total (32 × 32B desc + 32 × 1600B
/// buf for RX, plus 16 × 32B desc + 16 × 1600B buf for TX — the ESP32
/// EMAC enhanced descriptor layout is 8 dwords = 32 B per descriptor).
///
/// The default 10/10/1600 [`EmacDefault`] sizing is tuned for steady
/// production traffic where DMA latency budget is small and 32 KiB of
/// internal RAM is plenty. `EmacBench` deliberately over-provisions
/// the rings so the DMA missed-frame counter
/// ([`crate::regs::dma::missed_frames`]) stays at zero under burstier
/// senders (e.g. tight spin-poll loops), which makes the measured
/// throughput a property of the EMAC pipeline itself rather than of
/// ring depth.
///
/// # Memory budget — caller's responsibility
///
/// The 76.5 KiB sits in `.bss` (internal DRAM only — ESP32 EMAC DMA
/// is not PSRAM-capable on this silicon). Callers must verify their
/// linker layout has the headroom; a typical pattern is to drop a
/// `heap_allocator!` block of comparable size, or downsize to a
/// `Emac<16, 8, 1600>` (≈ 38 KiB) if the full depth isn't required.
pub type EmacBench = Emac<BENCH_RX, BENCH_TX, DEFAULT_BUF>;

// =============================================================================
// Helpers
// =============================================================================

/// Wraps a `&mut DelayNs` so it can be passed by value to APIs that take
/// an owned `DelayNs` implementor (such as
/// [`crate::reset::ResetController::with_timeout`]).
struct BorrowedDelay<'a, D: DelayNs + ?Sized>(&'a mut D);

impl<D: DelayNs + ?Sized> DelayNs for BorrowedDelay<'_, D> {
    fn delay_ns(&mut self, ns: u32) {
        self.0.delay_ns(ns);
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> EmacConfig {
        EmacConfig {
            clock: RmiiClockConfig::InternalApll {
                gpio: ClkGpio::Gpio17,
                xtal: crate::config::XtalFreq::Mhz40,
            },
            pins: crate::config::RmiiPins::default(),
        }
    }

    #[test]
    fn new_is_uninitialized() {
        let emac: EmacDefault = Emac::new(test_config());
        assert_eq!(emac.state(), EmacState::Uninitialized);
        assert_eq!(emac.mac_address(), [0u8; 6]);
    }

    #[test]
    fn set_mac_before_init_only_caches() {
        let mut emac: EmacDefault = Emac::new(test_config());
        let mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        emac.set_mac_address(mac);
        assert_eq!(emac.mac_address(), mac);
        // No register writes performed because state is Uninitialized.
    }

    #[test]
    fn memory_usage_matches_dma() {
        // Source the comparison from the same constants as the alias
        // itself — retuning `DEFAULT_*` continues to match without
        // touching this test.
        assert_eq!(
            EmacDefault::memory_usage(),
            DmaEngine::<DEFAULT_RX, DEFAULT_TX, DEFAULT_BUF>::memory_usage()
        );
    }
}
