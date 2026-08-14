// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! GPIO Matrix configuration for ESP32 EMAC SMI and RMII signals.
//!
//! Scope: **original ESP32 (Xtensa LX6) only.** Of the ESP32 family,
//! EMAC is present on the original ESP32 and on ESP32-P4 (the
//! S2/S3/C2/C3/C5/C6/H2 line has no EMAC at all). P4 is a RISC-V
//! chip with a different GPIO Matrix layout, a different IO_MUX
//! address space, and a newer Synopsys GMAC revision — supporting it
//! would require a chip-feature split through this module and is out
//! of scope today. All addresses, signal indices and the
//! `iomux_addr_for_gpio` lookup below are hard-wired to the original
//! ESP32 memory map; do not assume portability.
//!
//! Why direct register access instead of `esp_hal::gpio` connect APIs:
//! `esp-hal` 1.x has no `OutputSignal::EmacMdc` / `EmacMdo` /
//! `InputSignal::EmacMdi` variants — EMAC signals are not in the
//! enum at all because the peripheral isn't supported in the HAL.
//! Until that lands upstream, this module is the integration point.
//!
//! The SMI signals (MDC, MDIO) are routed through the GPIO Matrix, so
//! any GPIO with an IO_MUX register can be picked for them. The RMII
//! data pins are *not* routable — they are pinned to fixed GPIOs
//! (TXD0=19, TXD1=22, TX_EN=21, RXD0=25, RXD1=26, CRS_DV=27) and must
//! be selected through IO_MUX function 5.
//!
//! Signal table:
//!
//! | Signal       | Index | Direction | Default GPIO |
//! |--------------|-------|-----------|--------------|
//! | EMAC_MDC_O   | 200   | Output    | GPIO23       |
//! | EMAC_MDI_I   | 201   | Input     | GPIO18       |
//! | EMAC_MDO_O   | 201   | Output    | GPIO18       |

#![allow(dead_code)]

// =============================================================================
// GPIO peripheral
// =============================================================================

/// GPIO peripheral base address.
pub const GPIO_BASE: usize = 0x3FF4_4000;

/// `GPIO_OUT_W1TS` — output set (write-1-to-set), GPIO 0-31.
pub const GPIO_OUT_W1TS_OFFSET: usize = 0x08;
/// `GPIO_OUT_W1TC` — output clear (write-1-to-clear), GPIO 0-31.
pub const GPIO_OUT_W1TC_OFFSET: usize = 0x0C;
/// `GPIO_ENABLE_W1TS` — output-enable set (write-1-to-set), GPIO 0-31.
pub const GPIO_ENABLE_W1TS_OFFSET: usize = 0x24;
/// `GPIO_ENABLE_W1TC` — output-enable clear (write-1-to-clear), GPIO 0-31.
pub const GPIO_ENABLE_W1TC_OFFSET: usize = 0x28;
/// `GPIO_ENABLE1_W1TS` — output-enable set (write-1-to-set), GPIO 32-39.
/// Bit `N` in this register corresponds to GPIO `32 + N`.
pub const GPIO_ENABLE1_W1TS_OFFSET: usize = 0x30;
/// `GPIO_ENABLE1_W1TC` — output-enable clear (write-1-to-clear), GPIO 32-39.
/// Bit `N` in this register corresponds to GPIO `32 + N`.
pub const GPIO_ENABLE1_W1TC_OFFSET: usize = 0x34;
/// Base offset of `GPIO_FUNCx_IN_SEL_CFG_REG`. Per-signal stride 4.
pub const GPIO_FUNC_IN_SEL_CFG_BASE: usize = 0x130;
/// Base offset of `GPIO_FUNCx_OUT_SEL_CFG_REG`. Per-GPIO stride 4.
pub const GPIO_FUNC_OUT_SEL_CFG_BASE: usize = 0x530;

/// Signal index for the EMAC MDC output.
pub const EMAC_MDC_O_IDX: u32 = 200;
/// Signal index for the EMAC MDIO input.
pub const EMAC_MDI_I_IDX: u32 = 201;
/// Signal index for the EMAC MDIO output.
pub const EMAC_MDO_O_IDX: u32 = 201;

/// Function output select field mask in `GPIO_FUNCx_OUT_SEL_CFG_REG` (bits 8:0).
pub const GPIO_FUNC_OUT_SEL_MASK: u32 = 0x1FF;
/// Bit 10 of `GPIO_FUNCx_OUT_SEL_CFG_REG`: peripheral controls output enable.
pub const GPIO_OEN_SEL: u32 = 1 << 10;
/// "Disconnect" value for the output select field — routes the IO_MUX
/// peripheral function instead of any GPIO Matrix signal.
pub const GPIO_OUT_SEL_DISCONNECT: u32 = 256;
/// Function input select field mask in `GPIO_FUNCx_IN_SEL_CFG_REG` (bits 5:0).
pub const GPIO_FUNC_IN_SEL_MASK: u32 = 0x3F;
/// Bit 7 of `GPIO_FUNCx_IN_SEL_CFG_REG`: route through GPIO Matrix.
pub const GPIO_SIG_IN_SEL: u32 = 1 << 7;

// =============================================================================
// IO_MUX
// =============================================================================

/// IO_MUX base address.
pub const IO_MUX_BASE: usize = 0x3FF4_9000;
/// IO_MUX `MCU_SEL` field shift (bits 14:12).
pub const IO_MUX_MCU_SEL_SHIFT: u32 = 12;
/// IO_MUX `MCU_SEL` field mask.
pub const IO_MUX_MCU_SEL_MASK: u32 = 0x07 << 12;
/// `MCU_SEL=2` selects "GPIO Matrix" routing.
pub const IO_MUX_FUNC_GPIO: u32 = 2;
/// `MCU_SEL=5` selects EMAC peripheral function for fixed RMII pins.
pub const IO_MUX_FUNC_EMAC: u32 = 5;
/// IO_MUX `FUN_IE` (bit 9) — input buffer enable.
pub const IO_MUX_FUN_IE: u32 = 1 << 9;
/// IO_MUX `FUN_DRV` field shift (bits 11:10).
pub const IO_MUX_FUN_DRV_SHIFT: u32 = 10;
/// IO_MUX `FUN_DRV` field mask.
pub const IO_MUX_FUN_DRV_MASK: u32 = 0x03 << 10;

// =============================================================================
// Public configuration entry points
// =============================================================================

/// Route MDC and MDIO through the requested GPIOs via the GPIO Matrix.
/// Must be called before any MDIO transaction. The default EMAC bring-up
/// sequence picks these pins from [`crate::config::RmiiPins`].
///
/// Out-of-range GPIO numbers are silently ignored (early-return) so a
/// bad config can't write to unintended MMIO. Callers that want a hard
/// error should validate via [`is_valid_smi_pin`] first; `Emac::init`
/// already does so and returns `EmacError::InvalidConfig`.
pub fn configure_smi_pins(mdc_gpio: u8, mdio_gpio: u8) {
    if !is_valid_smi_pin(mdc_gpio) || !is_valid_smi_pin(mdio_gpio) {
        return;
    }
    configure_mdc(mdc_gpio);
    configure_mdio(mdio_gpio);
}

/// Returns `true` if `gpio_num` is a valid GPIO for SMI (MDC or MDIO)
/// routing on ESP32: the silicon must have an IO_MUX register for it
/// (per ESP32 TRM Table 4-3), and the pad must be output-capable
/// (rules out the input-only group GPIO34-39).
///
/// Accepted set is `{0..=23, 25..=27, 32..=33}`. Rejected:
///
/// - GPIO24 — no IO_MUX entry on any ESP32 die variant.
/// - GPIO28-31 — no IO_MUX entry, not present in the GPIO Matrix
///   layout (see `iomux_addr_for_gpio` lookup).
/// - GPIO34-39 — input-only on ESP32, so they cannot drive MDC and
///   cannot host bidirectional MDIO.
/// - Anything ≥ 40 — outside the documented GPIO range.
///
/// **Pad availability is package-dependent.** Some accepted GPIOs
/// (notably GPIO20) have an IO_MUX register on the silicon but are
/// not bonded to a pad on the standard QFN modules (`ESP32-WROOM-32`,
/// `ESP32-WROVER`, `ESP32-MINI`, etc.). The predicate intentionally
/// follows what the datasheet allows — bare-die / custom-bond designs
/// using e.g. GPIO20 for MDC are legal hardware and the driver must
/// not lock them out. Module / board-level pinout is the integrator's
/// responsibility, not this function's.
#[must_use]
pub const fn is_valid_smi_pin(gpio_num: u8) -> bool {
    matches!(gpio_num, 0..=23 | 25..=27 | 32..=33)
}

/// Route the six fixed RMII data pins through IO_MUX function 5
/// (TXD0/TXD1/TX_EN/RXD0/RXD1/CRS_DV). Must be called during EMAC init.
pub fn configure_rmii_pins() {
    // TX (output): GPIO19, GPIO22, GPIO21
    configure_iomux_output(19, IO_MUX_FUNC_EMAC);
    configure_iomux_output(22, IO_MUX_FUNC_EMAC);
    configure_iomux_output(21, IO_MUX_FUNC_EMAC);
    // RX (input): GPIO25, GPIO26, GPIO27
    configure_iomux_input(25, IO_MUX_FUNC_EMAC);
    configure_iomux_input(26, IO_MUX_FUNC_EMAC);
    configure_iomux_input(27, IO_MUX_FUNC_EMAC);
}

// =============================================================================
// MDC / MDIO routing through GPIO Matrix
// =============================================================================

fn configure_mdc(gpio_num: u8) {
    // SAFETY: all addresses are valid 32-bit ESP32 peripheral registers.
    unsafe {
        if let Some(iomux) = iomux_addr_for_gpio(gpio_num) {
            let cur = read_reg(iomux);
            let new_val = (cur & !IO_MUX_MCU_SEL_MASK) | (IO_MUX_FUNC_GPIO << IO_MUX_MCU_SEL_SHIFT);
            write_reg(iomux, new_val);
        }
        gpio_output_enable_set(gpio_num);
        // Matches ESP-IDF's `esp_rom_gpio_connect_out_signal(gpio, sig, false,
        // false)`: only the signal-select field is written, no `GPIO_OEN_SEL`.
        // Output-enable comes from the unconditional `gpio_output_enable_set`
        // call above (mirrors the ROM function's own unconditional
        // `GPIO_ENABLE_W1TS` write) — MDC never needs to tri-state.
        let out_sel = GPIO_BASE + GPIO_FUNC_OUT_SEL_CFG_BASE + (gpio_num as usize * 4);
        write_reg(out_sel, EMAC_MDC_O_IDX & GPIO_FUNC_OUT_SEL_MASK);
    }
}

fn configure_mdio(gpio_num: u8) {
    // SAFETY: all addresses are valid 32-bit ESP32 peripheral registers.
    unsafe {
        if let Some(iomux) = iomux_addr_for_gpio(gpio_num) {
            let cur = read_reg(iomux);
            let new_val = (cur & !IO_MUX_MCU_SEL_MASK)
                | (IO_MUX_FUNC_GPIO << IO_MUX_MCU_SEL_SHIFT)
                | IO_MUX_FUN_IE;
            write_reg(iomux, new_val);
        }
        gpio_output_enable_set(gpio_num);
        // GPIO output → EMAC_MDO_O. As in `configure_mdc`, this mirrors
        // ESP-IDF's `esp_rom_gpio_connect_out_signal` exactly (signal-select
        // field only, no `GPIO_OEN_SEL`) — setting `GPIO_OEN_SEL` here was
        // observed on real hardware to make every MDIO read come back as a
        // fixed 0 (ESP32 permanently driving the shared MDIO line, stomping
        // over the PHY's response) despite the MDIO transaction itself
        // completing (busy-bit clears, `GMACMIIADDR` reflects the correct
        // PHY/register address) — i.e. exactly the state-machine level looks
        // fine, only the physical bus contention is wrong.
        let out_sel = GPIO_BASE + GPIO_FUNC_OUT_SEL_CFG_BASE + (gpio_num as usize * 4);
        write_reg(out_sel, EMAC_MDO_O_IDX & GPIO_FUNC_OUT_SEL_MASK);
        // EMAC_MDI_I ← GPIO input.
        let in_sel = GPIO_BASE + GPIO_FUNC_IN_SEL_CFG_BASE + (EMAC_MDI_I_IDX as usize * 4);
        write_reg(
            in_sel,
            (gpio_num as u32 & GPIO_FUNC_IN_SEL_MASK) | GPIO_SIG_IN_SEL,
        );
    }
}

// =============================================================================
// IO_MUX direct routing (for the fixed RMII data pins)
// =============================================================================

fn configure_iomux_output(gpio_num: u8, func: u32) {
    let Some(iomux) = iomux_addr_for_gpio(gpio_num) else {
        return;
    };
    // SAFETY: IO_MUX[gpio] and the GPIO Matrix output-sel register are valid.
    unsafe {
        let cur = read_reg(iomux);
        // Clear MCU_SEL/FUN_IE/FUN_DRV/pull-up/pull-down, set MCU_SEL=func,
        // set FUN_DRV=3 (max).
        let new_val = (cur
            & !IO_MUX_MCU_SEL_MASK
            & !(1 << 7)
            & !(1 << 8)
            & !IO_MUX_FUN_IE
            & !IO_MUX_FUN_DRV_MASK)
            | (func << IO_MUX_MCU_SEL_SHIFT)
            | (3 << IO_MUX_FUN_DRV_SHIFT);
        write_reg(iomux, new_val);
        // Disconnect any GPIO Matrix output mapped to this pin.
        let out_sel = GPIO_BASE + GPIO_FUNC_OUT_SEL_CFG_BASE + (gpio_num as usize * 4);
        write_reg(out_sel, GPIO_OUT_SEL_DISCONNECT);
    }
}

fn configure_iomux_input(gpio_num: u8, func: u32) {
    let Some(iomux) = iomux_addr_for_gpio(gpio_num) else {
        return;
    };
    // SAFETY: IO_MUX[gpio] and the GPIO Matrix output-sel register are valid.
    unsafe {
        let cur = read_reg(iomux);
        // Clear MCU_SEL/pull-up/pull-down, set MCU_SEL=func, enable input.
        let new_val = (cur & !IO_MUX_MCU_SEL_MASK & !(1 << 7) & !(1 << 8))
            | (func << IO_MUX_MCU_SEL_SHIFT)
            | IO_MUX_FUN_IE;
        write_reg(iomux, new_val);
        // Disconnect any GPIO Matrix output mapped to this pin.
        let out_sel = GPIO_BASE + GPIO_FUNC_OUT_SEL_CFG_BASE + (gpio_num as usize * 4);
        write_reg(out_sel, GPIO_OUT_SEL_DISCONNECT);
    }
}

// =============================================================================
// Helpers
// =============================================================================

#[inline(always)]
unsafe fn read_reg(addr: usize) -> u32 {
    // SAFETY: caller guarantees address validity.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(always)]
unsafe fn write_reg(addr: usize, val: u32) {
    // SAFETY: caller guarantees address validity.
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

/// Set the output-enable bit for `gpio_num` via the appropriate
/// `GPIO_ENABLE*_W1TS_REG`. Splits into the upper bank (`GPIO_ENABLE1`)
/// for GPIO 32-39, where a `1u32 << gpio_num` shift in the lower-bank
/// register would either alias another GPIO or invoke shift-overflow UB.
///
/// # Safety
///
/// Writes to the GPIO peripheral. Caller must ensure `gpio_num <= 39`
/// (the only physical range on ESP32). Out-of-range numbers are a no-op.
#[inline]
unsafe fn gpio_output_enable_set(gpio_num: u8) {
    // SAFETY: GPIO_BASE + offset is a known-valid 32-bit register.
    unsafe {
        if gpio_num < 32 {
            write_reg(GPIO_BASE + GPIO_ENABLE_W1TS_OFFSET, 1u32 << gpio_num);
        } else if gpio_num < 40 {
            write_reg(
                GPIO_BASE + GPIO_ENABLE1_W1TS_OFFSET,
                1u32 << (gpio_num - 32),
            );
        }
    }
}

/// IO_MUX register address for a given GPIO. Per ESP32 TRM Table 4-3 the
/// IO_MUX layout is non-sequential, so we have an explicit lookup.
/// Returns `None` for GPIOs that have no IO_MUX register: GPIO24 has no
/// pad on any ESP32 package, and any number outside the documented
/// range (above 39) is rejected.
fn iomux_addr_for_gpio(gpio_num: u8) -> Option<usize> {
    let offset = match gpio_num {
        0 => 0x44,
        1 => 0x88,
        2 => 0x40,
        3 => 0x84,
        4 => 0x48,
        5 => 0x6C,
        6 => 0x60,
        7 => 0x64,
        8 => 0x68,
        9 => 0x54,
        10 => 0x58,
        11 => 0x5C,
        12 => 0x34,
        13 => 0x38,
        14 => 0x30,
        15 => 0x3C,
        16 => 0x4C,
        17 => 0x50,
        18 => 0x70,
        19 => 0x74,
        20 => 0x78,
        21 => 0x7C,
        22 => 0x80,
        23 => 0x8C,
        25 => 0x24,
        26 => 0x28,
        27 => 0x2C,
        32 => 0x1C,
        33 => 0x20,
        34 => 0x14,
        35 => 0x18,
        36 => 0x04,
        37 => 0x08,
        38 => 0x0C,
        39 => 0x10,
        _ => return None,
    };
    Some(IO_MUX_BASE + offset)
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_indices() {
        assert_eq!(EMAC_MDC_O_IDX, 200);
        assert_eq!(EMAC_MDI_I_IDX, 201);
        assert_eq!(EMAC_MDO_O_IDX, 201);
    }

    #[test]
    fn out_sel_address_gpio23() {
        let addr = GPIO_BASE + GPIO_FUNC_OUT_SEL_CFG_BASE + (23 * 4);
        assert_eq!(addr, 0x3FF4_458C);
    }

    #[test]
    fn in_sel_address_emac_mdi() {
        let addr = GPIO_BASE + GPIO_FUNC_IN_SEL_CFG_BASE + (EMAC_MDI_I_IDX as usize * 4);
        assert_eq!(addr, 0x3FF4_4454);
    }

    #[test]
    fn iomux_addresses_for_smi() {
        assert_eq!(iomux_addr_for_gpio(18), Some(0x3FF4_9070));
        assert_eq!(iomux_addr_for_gpio(23), Some(0x3FF4_908C));
    }

    #[test]
    fn iomux_addr_for_gpio20_is_known() {
        // GPIO20 has an IO_MUX register on ESP32 (`IO_MUX_GPIO20_REG` at
        // `IO_MUX_BASE + 0x78`). It isn't bonded on the standard QFN
        // modules (WROOM/WROVER/MINI), but bare-die / custom designs
        // can route it — the lookup follows the silicon, not module
        // pinouts.
        assert_eq!(iomux_addr_for_gpio(20), Some(0x3FF4_9078));
    }

    #[test]
    fn iomux_addr_for_gpio_out_of_range_is_none() {
        assert_eq!(iomux_addr_for_gpio(24), None);
        assert_eq!(iomux_addr_for_gpio(40), None);
    }

    #[test]
    fn smi_pin_validation() {
        // Defaults must be accepted.
        assert!(is_valid_smi_pin(23));
        assert!(is_valid_smi_pin(18));
        // GPIO0 is sometimes used as a strapping pin but technically valid.
        assert!(is_valid_smi_pin(0));
        // Boundaries of the lower bank.
        assert!(is_valid_smi_pin(23));
        assert!(is_valid_smi_pin(25));
        assert!(is_valid_smi_pin(27));
        // GPIO24 has no pad / IO_MUX entry on any ESP32 package.
        assert!(!is_valid_smi_pin(24));
        // GPIO28-31 are not bonded and have no IO_MUX entry.
        assert!(!is_valid_smi_pin(28));
        assert!(!is_valid_smi_pin(31));
        // Boundary of the output-capable upper bank.
        assert!(is_valid_smi_pin(32));
        assert!(is_valid_smi_pin(33));
        // GPIO34-39 are input-only on ESP32.
        assert!(!is_valid_smi_pin(34));
        assert!(!is_valid_smi_pin(39));
        // Out-of-range.
        assert!(!is_valid_smi_pin(40));
        assert!(!is_valid_smi_pin(255));
    }

    #[test]
    fn enable_register_offsets() {
        // Per ESP32 TRM section 4.10 ("GPIO Matrix and IO_MUX") and
        // esp-idf `soc/gpio_reg.h`: GPIO 0-31 use the W1TS at +0x24,
        // GPIO 32-39 use the upper-bank W1TS at +0x30.
        assert_eq!(GPIO_BASE + GPIO_ENABLE_W1TS_OFFSET, 0x3FF4_4024);
        assert_eq!(GPIO_BASE + GPIO_ENABLE_W1TC_OFFSET, 0x3FF4_4028);
        assert_eq!(GPIO_BASE + GPIO_ENABLE1_W1TS_OFFSET, 0x3FF4_4030);
        assert_eq!(GPIO_BASE + GPIO_ENABLE1_W1TC_OFFSET, 0x3FF4_4034);
    }
}
