// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! APLL 50 MHz clock configuration and GPIO clock output/input setup.
//!
//! The ESP32 EMAC RMII interface requires a 50 MHz reference clock.
//! It can be generated internally by the Audio PLL (APLL) or supplied
//! externally from the PHY crystal oscillator.
//!
//! ## Internal APLL mode
//!
//! 1. [`configure_apll_50mhz`] powers up APLL and programs its coefficients
//!    via ROM I2C to produce 50 MHz from the 40 MHz XTAL.
//! 2. [`configure_emac_clk_out`] sets up a GPIO (0, 16, or 17) as clock
//!    output via IO_MUX function 5 so the PHY receives 50 MHz.
//!
//! The EMAC_EXT clock path registers (int_en, clk_sel, clk_en) are
//! configured separately by [`Emac::init`](crate::emac::Emac::init)
//! via `configure_phy_interface()` and `enable_ext_clocks()`.
//!
//! ## External clock mode
//!
//! [`configure_emac_clk_in`] sets up a GPIO as clock input via IO_MUX.
//! The EMAC_EXT registers for external mode are handled by `Emac::init`.
//!
//! ## APLL/WiFi conflict
//!
//! APLL cannot coexist with WiFi/BT (ESP32 errata CLK-3.22).
//! Use external clock when Ethernet + WiFi is needed.
//!
//! ## ROM I2C details
//!
//! esp-hal does not yet expose APLL configuration (its `soc/esp32/clocks.rs`
//! has `todo!()`). We use the ROM I2C functions directly:
//! - APLL I2C block ID: `0x6D`, host ID: **3** (verified on hardware).
//! - ANA_CONF register (`0x3FF4_8030`): bit 24 = PU, bit 23 = PD.

use crate::config::{ClkGpio, XtalFreq};

// =============================================================================
// APLL ROM I2C constants
// =============================================================================

/// APLL I2C block identifier for ROM I2C functions.
const I2C_APLL: u8 = 0x6D;

/// APLL I2C host identifier (ESP32-specific, verified on hardware).
///
/// ESP-IDF headers suggest 0 or 4, but hardware testing confirmed
/// host ID 3 is correct for ESP32 APLL access.
const I2C_APLL_HOSTID: u8 = 3;

/// RTC analog configuration register address.
///
/// Contains APLL power-up (bit 24) and power-down (bit 23) controls.
/// From ESP32 SVD: `RTC_CNTL_ANA_CONF_REG`.
const ANA_CONF_REG: usize = 0x3FF4_8030;

/// APLL force power-up bit in ANA_CONF (bit 24).
const ANA_CONF_PLLA_FORCE_PU: u32 = 1 << 24;

/// APLL force power-down bit in ANA_CONF (bit 23).
const ANA_CONF_PLLA_FORCE_PD: u32 = 1 << 23;

// =============================================================================
// GPIO/IO_MUX constants
// =============================================================================

/// IO_MUX base address (ESP32).
const IO_MUX_BASE: usize = 0x3FF4_9000;

/// GPIO peripheral base address.
const GPIO_BASE: usize = 0x3FF4_4000;

/// GPIO output function select register base offset.
/// For GPIO N: `GPIO_BASE + 0x530 + N*4`.
const GPIO_FUNC_OUT_SEL_BASE: usize = GPIO_BASE + 0x530;

/// GPIO output enable set (write-1-to-set) register.
const GPIO_ENABLE_W1TS: usize = GPIO_BASE + 0x024;

/// IO_MUX MCU_SEL field mask (bits 14:12).
const MCU_SEL_MASK: u32 = 0x7 << 12;

/// IO_MUX FUN_DRV (drive strength) field mask (bits 11:10).
const FUN_DRV_MASK: u32 = 0x3 << 10;

/// IO_MUX FUN_IE (input enable) bit 9.
const FUN_IE: u32 = 1 << 9;

/// Number of spin-loop iterations to wait after APLL power-up.
///
/// Matches the firmware reference. Provides ~10-20 us settling time
/// at typical ESP32 CPU frequencies (160-240 MHz).
const APLL_POWER_UP_SPIN: u32 = 10_000;

// =============================================================================
// ROM I2C FFI
// =============================================================================

unsafe extern "C" {
    fn rom_i2c_writeReg(block: u8, block_hostid: u8, reg_add: u8, indata: u8);
    fn rom_i2c_readReg(block: u8, block_hostid: u8, reg_add: u8) -> u8;
}

/// Read an APLL register via ROM I2C.
#[inline(always)]
fn regi2c_read(reg: u8) -> u8 {
    // SAFETY: ROM I2C functions are always available on ESP32.
    unsafe { rom_i2c_readReg(I2C_APLL, I2C_APLL_HOSTID, reg) }
}

/// Write an APLL register via ROM I2C.
#[inline(always)]
fn regi2c_write(reg: u8, data: u8) {
    // SAFETY: ROM I2C functions are always available on ESP32.
    unsafe { rom_i2c_writeReg(I2C_APLL, I2C_APLL_HOSTID, reg, data) }
}

/// Masked write to an APLL register: modify bits `[msb:lsb]` to `val`.
fn apll_write_mask(reg: u8, msb: u8, lsb: u8, val: u8) {
    let old = regi2c_read(reg);
    let mask = ((1u16 << (msb - lsb + 1)) - 1) as u8;
    let new = (old & !(mask << lsb)) | ((val & mask) << lsb);
    regi2c_write(reg, new);
}

// =============================================================================
// Public API
// =============================================================================

/// SDM coefficients for the ESP32 APLL.
///
/// Output frequency formula:
///
/// ```text
/// fout = fxtal * (sdm2 + sdm1/256 + sdm0/65536 + 4) / (2 * (o_div + 2))
/// ```
///
/// For each supported on-board crystal, [`ApllCoefficients::for_xtal`]
/// returns the coefficients that land on **50 MHz** (the RMII reference
/// clock).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApllCoefficients {
    /// Fine fractional multiplier (×1/65536). 8-bit field.
    pub sdm0: u8,
    /// Mid fractional multiplier (×1/256). 8-bit field.
    pub sdm1: u8,
    /// Integer-part multiplier (added to fixed +4). 6-bit field
    /// (`apll_write_mask(7, 5, 0, sdm2)`).
    pub sdm2: u8,
    /// Output divider field. Final divisor is `2 * (o_div + 2)`. 5-bit
    /// field (`apll_write_mask(4, 4, 0, o_div)`).
    pub o_div: u8,
}

impl ApllCoefficients {
    /// Look up the coefficients that produce a 50 MHz APLL output for
    /// the given on-board crystal.
    ///
    /// Total: infallible — the input is constrained by [`XtalFreq`],
    /// which only enumerates crystals the crate has verified
    /// coefficients for (`Mhz26` / `Mhz32` / `Mhz40`). Adding support
    /// for another crystal therefore takes two concrete edits — extend
    /// `XtalFreq` with the new variant, and add a matching arm here —
    /// followed by a host-side unit test asserting the new arm lands
    /// on 50 MHz.
    ///
    /// Verified results (target 50.000 MHz):
    ///
    /// | XTAL  | sdm2 | sdm1 | sdm0 | o_div | Computed fout |
    /// |-------|------|------|------|-------|---------------|
    /// | 26 MHz| 11   | 98   | 118  | 2     | 50.0000 MHz   |
    /// | 32 MHz| 8    | 128  | 0    | 2     | 50.0000 MHz   |
    /// | 40 MHz| 6    | 0    | 0    | 2     | 50.0000 MHz   |
    pub const fn for_xtal(xtal: XtalFreq) -> Self {
        match xtal {
            // 50 MHz = 26 MHz * (11 + 98/256 + 118/65536 + 4) / 8
            XtalFreq::Mhz26 => Self {
                sdm0: 118,
                sdm1: 98,
                sdm2: 11,
                o_div: 2,
            },
            // 50 MHz = 32 MHz * (8 + 128/256 + 0/65536 + 4) / 8
            XtalFreq::Mhz32 => Self {
                sdm0: 0,
                sdm1: 128,
                sdm2: 8,
                o_div: 2,
            },
            // 50 MHz = 40 MHz * (6 + 0 + 0 + 4) / 8
            XtalFreq::Mhz40 => Self {
                sdm0: 0,
                sdm1: 0,
                sdm2: 6,
                o_div: 2,
            },
        }
    }
}

/// Configure ESP32 APLL to output 50 MHz for EMAC RMII clock,
/// using SDM coefficients chosen for the on-board crystal.
///
/// APLL formula: `fout = fxtal * (sdm2 + sdm1/256 + sdm0/65536 + 4) / (2 * (o_div + 2))`.
/// See [`ApllCoefficients::for_xtal`] for the per-crystal table.
///
/// This function:
/// 1. Powers up APLL via ANA_CONF register
/// 2. Programmes SDM coefficients (`sdm2`/`sdm1`/`sdm0`/`o_div`) for the
///    requested crystal
/// 3. Runs the calibration sequence (from ESP-IDF `clk_ll_apll_set_config`)
///
/// The EMAC_EXT clock path registers (RMII mode, int_en, clk_sel) are
/// configured separately by [`Emac::init`](crate::emac::Emac::init).
///
/// # Ordering
///
/// Independent of the EMAC peripheral clock — the routine only writes
/// RTC analog registers (`ANA_CONF`) and APLL coefficients via the ROM
/// I2C controller, both of which sit on the always-on APB clock from
/// XTAL/main PLL. May be called before or after
/// `ext::enable_peripheral_clock`. Only required when the MCU is the
/// RMII clock master (i.e. `RmiiClockConfig::InternalApll`); skip it
/// entirely for `RmiiClockConfig::External`.
///
/// # Safety
///
/// Writes to RTC analog registers and APLL coefficients via ROM I2C.
/// Don't call concurrently with other RTC analog reconfiguration.
pub fn configure_apll_50mhz(xtal: XtalFreq) {
    let c = ApllCoefficients::for_xtal(xtal);

    // Step 1: Power up APLL
    // ANA_CONF: clear PD (bit 23), set PU (bit 24)
    unsafe {
        let ana = core::ptr::read_volatile(ANA_CONF_REG as *const u32);
        core::ptr::write_volatile(
            ANA_CONF_REG as *mut u32,
            (ana & !ANA_CONF_PLLA_FORCE_PD) | ANA_CONF_PLLA_FORCE_PU,
        );
    }
    // Wait for APLL to stabilize.
    for _ in 0..APLL_POWER_UP_SPIN {
        core::hint::spin_loop();
    }

    // Step 2: APLL coefficients — chosen by `for_xtal`.
    apll_write_mask(7, 5, 0, c.sdm2);
    apll_write_mask(9, 7, 0, c.sdm0);
    apll_write_mask(8, 7, 0, c.sdm1);

    // Step 3: Calibration sequence (from ESP-IDF clk_ll_apll_set_config)
    regi2c_write(5, 0x09);
    regi2c_write(5, 0x49);
    apll_write_mask(4, 4, 0, c.o_div);
    regi2c_write(0, 0x0F);
    regi2c_write(0, 0x3F);
    regi2c_write(0, 0x1F);
}

/// Configure a GPIO as EMAC 50 MHz RMII clock output via IO_MUX function 5.
///
/// On ESP32, only GPIO0, GPIO16, and GPIO17 support EMAC clock output:
/// - GPIO0:  `EMAC_TX_CLK` (also boot strapping -- use with caution)
/// - GPIO16: `EMAC_CLK_OUT` (0 degree phase)
/// - GPIO17: `EMAC_CLK_OUT_180` (180 degree phase, most common for LAN8720A)
///
/// Sets IO_MUX to function 5 with maximum drive strength, disconnects
/// the GPIO Matrix (IO_MUX direct), and enables the output driver.
///
/// # Safety
///
/// Writes to IO_MUX and GPIO registers. Must be called before DMA reset.
pub fn configure_emac_clk_out(gpio: ClkGpio) {
    let io_mux_addr = io_mux_addr_for_clk_gpio(gpio);
    let gpio_num = gpio.gpio_num() as usize;

    unsafe {
        // Set IO_MUX function 5 (EMAC clock) + maximum drive strength (3).
        let val = core::ptr::read_volatile(io_mux_addr as *const u32);
        core::ptr::write_volatile(
            io_mux_addr as *mut u32,
            (val & !MCU_SEL_MASK & !FUN_DRV_MASK) | (5 << 12) | (3 << 10),
        );

        // Disconnect GPIO Matrix -- use IO_MUX directly.
        // Writing 256 (SIG_GPIO_OUT_IDX) disconnects the Matrix output.
        core::ptr::write_volatile((GPIO_FUNC_OUT_SEL_BASE + gpio_num * 4) as *mut u32, 256);

        // Enable output driver.
        core::ptr::write_volatile(GPIO_ENABLE_W1TS as *mut u32, 1u32 << gpio_num);
    }
}

/// `PIN_CTRL` register — selects the clock source for the three SoC-wide
/// "clock output" channels (`CLK_OUT1/2/3`), independent of the EMAC
/// peripheral's own IO_MUX function 5. From ESP32 TRM / `soc/gpio_reg.h`.
const PIN_CTRL_REG: usize = IO_MUX_BASE;

/// `CLK_OUT1` field in `PIN_CTRL` (bits 3:0) — selects which internal
/// clock signal is routed to `CLKOUT_CHANNEL_1`, which is hard-wired to
/// GPIO0 (`soc/clkout_channel.h`: `CLKOUT_CHANNEL1_GPIO = GPIO0`).
const PIN_CTRL_CLK_OUT1_MASK: u32 = 0xF;
/// `CLK_OUT2` field in `PIN_CTRL` (bits 7:4). Unused by this driver, but
/// must be cleared — see [`configure_apll_clkout_gpio0`] doc comment on
/// the ESP32 `PIN_CTRL` reset-default silicon bug.
const PIN_CTRL_CLK_OUT2_MASK: u32 = 0xF << 4;
/// `CLK_OUT3` field in `PIN_CTRL` (bits 11:8). Unused by this driver, but
/// must be cleared — see [`configure_apll_clkout_gpio0`] doc comment.
const PIN_CTRL_CLK_OUT3_MASK: u32 = 0xF << 8;

/// Clock-output mux source id for the APLL (`soc/clk_tree_defs.h`:
/// `CLKOUT_SIG_APLL = 6`).
const CLKOUT_SIG_APLL: u32 = 6;

/// GPIO0 IO_MUX `MCU_SEL` value that routes `CLKOUT_CHANNEL_1` to the pad
/// (`soc/clkout_channel.h`: `FUNC_GPIO0_CLK_OUT1 = FUNC_GPIO0_CLK_OUT1`,
/// i.e. IO_MUX function 1). This is deliberately *not* function 5 — that
/// value selects `EMAC_TX_CLK`, which is an input-only net for GPIO0.
const IO_MUX_FUNC_CLK_OUT1: u32 = 1;

/// Configure GPIO0 to output the APLL-driven 50 MHz RMII reference clock
/// via the SoC clock-output mux (`PIN_CTRL.CLK_OUT1` → `CLKOUT_CHANNEL_1`
/// → GPIO0 IO_MUX function 1), for [`crate::config::RmiiClockConfig::InternalApllClkOutGpio0`].
///
/// Unlike [`configure_emac_clk_out`] (GPIO16/17, EMAC-dedicated IO_MUX
/// function 5), GPIO0 has no such dedicated EMAC clock-output net —
/// function 5 on GPIO0 is `EMAC_TX_CLK`, an input. This routes the APLL
/// output through the independent SoC-wide clock-output mux instead,
/// mirroring ESP-IDF's `esp_clock_output_start(CLKOUT_SIG_APLL, GPIO0, ..)`
/// path (`esp_eth_mac_esp.c`, `CONFIG_ETH_RMII_CLK_OUTPUT_GPIO0`).
///
/// Call [`configure_apll_50mhz`] first (same precondition as the
/// `InternalApll` path: APLL must be running before the DMA software
/// reset). The EMAC-internal clock-source-select register is still
/// programmed as "internal" by `Emac::init`, identical to the GPIO16/17
/// case — only the pad routing differs.
///
/// # ESP32 `PIN_CTRL` silicon bug
///
/// `PIN_CTRL`'s reset default is `0xFFF` — all three `CLK_OUTn` fields at
/// `0xF`. Per ESP-IDF (`esp_clock_output.c`, `esp_clock_output_pin_ctrl_init`,
/// run as a `__attribute__((constructor))` on every ESP32 boot): "Due to a
/// hardware bug, `PIN_CTRL` cannot select `0xf` output, whereas `0xf` is the
/// default value." ESP-IDF zeroes **all three** `CLK_OUTn` fields before
/// configuring any of them. This driver has no equivalent global startup
/// hook, so it clears all three fields here, immediately before
/// programming `CLK_OUT1` — leaving `CLK_OUT2`/`CLK_OUT3` at the buggy
/// `0xF` default while only patching `CLK_OUT1` was observed to leave the
/// RMII reference clock silent on real hardware even though the `CLK_OUT1`
/// field itself was set correctly.
///
/// # Safety
///
/// Writes to IO_MUX, `PIN_CTRL`, and GPIO registers. Must be called before
/// DMA reset.
pub fn configure_apll_clkout_gpio0() {
    unsafe {
        // Clear all three CLK_OUTn fields first (silicon-bug workaround,
        // see doc comment above), then route the APLL clock signal to
        // CLKOUT_CHANNEL_1 (fixed to GPIO0).
        let pin_ctrl = core::ptr::read_volatile(PIN_CTRL_REG as *const u32);
        let cleared = pin_ctrl
            & !PIN_CTRL_CLK_OUT1_MASK
            & !PIN_CTRL_CLK_OUT2_MASK
            & !PIN_CTRL_CLK_OUT3_MASK;
        core::ptr::write_volatile(
            PIN_CTRL_REG as *mut u32,
            cleared | (CLKOUT_SIG_APLL & PIN_CTRL_CLK_OUT1_MASK),
        );

        // GPIO0 IO_MUX: function 1 (CLK_OUT1), max drive strength, output
        // (not input) — mirrors `configure_emac_clk_out` but with function
        // value 1 instead of 5.
        let io_mux_addr = io_mux_addr_for_clk_gpio(ClkGpio::Gpio0);
        let val = core::ptr::read_volatile(io_mux_addr as *const u32);
        core::ptr::write_volatile(
            io_mux_addr as *mut u32,
            (val & !MCU_SEL_MASK & !FUN_DRV_MASK & !FUN_IE)
                | (IO_MUX_FUNC_CLK_OUT1 << 12)
                | (3 << 10),
        );

        // Disconnect GPIO Matrix -- use IO_MUX directly.
        core::ptr::write_volatile((GPIO_FUNC_OUT_SEL_BASE + 0 * 4) as *mut u32, 256);

        // Enable output driver on GPIO0.
        core::ptr::write_volatile(GPIO_ENABLE_W1TS as *mut u32, 1u32);
    }
}

/// Configure a GPIO as EMAC external 50 MHz clock input via IO_MUX.
///
/// Sets IO_MUX to function 5 with input enabled. Disconnects GPIO Matrix
/// to ensure IO_MUX is used directly.
///
/// Typically GPIO0 (`EMAC_TX_CLK` / RMII ref clock input).
///
/// # Safety
///
/// Writes to IO_MUX and GPIO registers. Must be called before DMA reset.
pub fn configure_emac_clk_in(gpio: ClkGpio) {
    let io_mux_addr = io_mux_addr_for_clk_gpio(gpio);
    let gpio_num = gpio.gpio_num() as usize;

    unsafe {
        // Set IO_MUX function 5 (EMAC clock) + input enable.
        let val = core::ptr::read_volatile(io_mux_addr as *const u32);
        core::ptr::write_volatile(
            io_mux_addr as *mut u32,
            (val & !MCU_SEL_MASK) | (5 << 12) | FUN_IE,
        );

        // Disconnect GPIO Matrix output -- use IO_MUX directly.
        core::ptr::write_volatile((GPIO_FUNC_OUT_SEL_BASE + gpio_num * 4) as *mut u32, 256);
    }
}

// =============================================================================
// Helpers
// =============================================================================

/// Return the IO_MUX register address for a clock-capable GPIO.
///
/// Based on ESP32 TRM Table 4-3:
/// - GPIO0:  offset 0x44
/// - GPIO16: offset 0x4C
/// - GPIO17: offset 0x50
const fn io_mux_addr_for_clk_gpio(gpio: ClkGpio) -> usize {
    let offset = match gpio {
        ClkGpio::Gpio0 => 0x44,
        ClkGpio::Gpio16 => 0x4C,
        ClkGpio::Gpio17 => 0x50,
    };
    IO_MUX_BASE + offset
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pin_ctrl_reg_is_io_mux_base() {
        // PIN_CTRL is IO_MUX offset 0x00 (ESP32 TRM).
        assert_eq!(PIN_CTRL_REG, IO_MUX_BASE);
    }

    #[test]
    fn pin_ctrl_clk_out_field_masks_are_disjoint() {
        // Each CLK_OUTn field is 4 bits wide and must not overlap its
        // neighbours, so the silicon-bug workaround (clearing all three
        // before programming CLK_OUT1) can't accidentally corrupt a field
        // it didn't mean to touch.
        assert_eq!(PIN_CTRL_CLK_OUT1_MASK & PIN_CTRL_CLK_OUT2_MASK, 0);
        assert_eq!(PIN_CTRL_CLK_OUT2_MASK & PIN_CTRL_CLK_OUT3_MASK, 0);
        assert_eq!(PIN_CTRL_CLK_OUT1_MASK & PIN_CTRL_CLK_OUT3_MASK, 0);
        assert_eq!(PIN_CTRL_CLK_OUT1_MASK, 0xF);
        assert_eq!(PIN_CTRL_CLK_OUT2_MASK, 0xF0);
        assert_eq!(PIN_CTRL_CLK_OUT3_MASK, 0xF00);
    }

    #[test]
    fn clkout_sig_apll_value() {
        // soc/clk_tree_defs.h: CLKOUT_SIG_APLL = 6.
        assert_eq!(CLKOUT_SIG_APLL, 6);
        assert_eq!(CLKOUT_SIG_APLL & PIN_CTRL_CLK_OUT1_MASK, CLKOUT_SIG_APLL);
    }

    #[test]
    fn io_mux_func_clk_out1_is_not_emac_func5() {
        // FUNC_GPIO0_CLK_OUT1 = 1, distinct from IO_MUX_FUNC_EMAC = 5 used
        // by `configure_emac_clk_out`/`configure_emac_clk_in` on GPIO16/17.
        assert_eq!(IO_MUX_FUNC_CLK_OUT1, 1);
        assert_ne!(IO_MUX_FUNC_CLK_OUT1, 5);
    }

    #[test]
    fn clk_gpio_io_mux_addresses() {
        // Verify IO_MUX offsets match the ESP32 TRM pad list.
        assert_eq!(
            io_mux_addr_for_clk_gpio(ClkGpio::Gpio0),
            0x3FF4_9044,
            "GPIO0 IO_MUX address mismatch"
        );
        assert_eq!(
            io_mux_addr_for_clk_gpio(ClkGpio::Gpio16),
            0x3FF4_904C,
            "GPIO16 IO_MUX address mismatch"
        );
        assert_eq!(
            io_mux_addr_for_clk_gpio(ClkGpio::Gpio17),
            0x3FF4_9050,
            "GPIO17 IO_MUX address mismatch"
        );
    }

    #[test]
    fn clk_gpio_numbers_match_enum() {
        assert_eq!(ClkGpio::Gpio0.gpio_num(), 0);
        assert_eq!(ClkGpio::Gpio16.gpio_num(), 16);
        assert_eq!(ClkGpio::Gpio17.gpio_num(), 17);
    }

    #[test]
    fn ana_conf_bits_no_overlap() {
        assert_eq!(
            ANA_CONF_PLLA_FORCE_PU & ANA_CONF_PLLA_FORCE_PD,
            0,
            "PU and PD bits must not overlap"
        );
    }

    #[test]
    fn ana_conf_bit_positions() {
        // PD = bit 23, PU = bit 24
        assert_eq!(ANA_CONF_PLLA_FORCE_PD, 1 << 23);
        assert_eq!(ANA_CONF_PLLA_FORCE_PU, 1 << 24);
    }

    #[test]
    fn ana_conf_register_address() {
        assert_eq!(ANA_CONF_REG, 0x3FF4_8030);
    }

    #[test]
    fn apll_constants() {
        assert_eq!(I2C_APLL, 0x6D);
        assert_eq!(I2C_APLL_HOSTID, 3);
    }

    #[test]
    fn io_mux_base_consistent_with_ext_regs() {
        assert_eq!(IO_MUX_BASE, crate::regs::ext::IO_MUX_BASE);
    }

    #[test]
    fn gpio_register_layout() {
        // GPIO_FUNC_OUT_SEL for GPIO0 should be at GPIO_BASE + 0x530
        assert_eq!(GPIO_FUNC_OUT_SEL_BASE, 0x3FF4_4530);
        // GPIO_ENABLE_W1TS should be at GPIO_BASE + 0x024
        assert_eq!(GPIO_ENABLE_W1TS, 0x3FF4_4024);
    }

    #[test]
    fn mcu_sel_mask_covers_function_5() {
        // Function 5 = 0b101, fits in 3-bit MCU_SEL field at bits 14:12
        let func5_shifted = 5u32 << 12;
        assert_eq!(func5_shifted & MCU_SEL_MASK, func5_shifted);
    }

    #[test]
    fn fun_drv_max_strength() {
        // Max drive strength = 3, shifted to bits 11:10
        let max_drv = 3u32 << 10;
        assert_eq!(max_drv & FUN_DRV_MASK, max_drv);
    }

    // ── APLL coefficients ────────────────────────────────────────────────

    /// Compute output frequency in MHz·Q16 fixed-point from APLL
    /// coefficients, for a host-side sanity check that the table really
    /// lands on 50 MHz. Matches the silicon formula:
    ///   fout = fxtal * (sdm2 + sdm1/256 + sdm0/65536 + 4) / (2 * (o_div + 2))
    fn fout_mhz_q16(c: ApllCoefficients, xtal_mhz: u32) -> u64 {
        let num = (xtal_mhz as u64)
            * (((c.sdm2 as u64 + 4) << 16) + (c.sdm1 as u64 * 256) + c.sdm0 as u64);
        let denom = 2 * (c.o_div as u64 + 2);
        num / denom
    }

    fn assert_50mhz(c: ApllCoefficients, xtal_mhz: u32) {
        let q16 = fout_mhz_q16(c, xtal_mhz);
        // 50 MHz in Q16: 50 << 16 = 3_276_800.
        let target_q16 = 50u64 << 16;
        // Allow ±0.001 MHz drift.
        let drift = q16.abs_diff(target_q16);
        assert!(
            drift < 100,
            "fout for {} MHz XTAL is {} (Q16) — drift {} from 50 MHz target",
            xtal_mhz,
            q16,
            drift
        );
    }

    #[test]
    fn apll_coefficients_xtal_40_lands_on_50mhz() {
        assert_50mhz(ApllCoefficients::for_xtal(XtalFreq::Mhz40), 40);
    }

    #[test]
    fn apll_coefficients_xtal_32_lands_on_50mhz() {
        assert_50mhz(ApllCoefficients::for_xtal(XtalFreq::Mhz32), 32);
    }

    #[test]
    fn apll_coefficients_xtal_26_lands_on_50mhz() {
        assert_50mhz(ApllCoefficients::for_xtal(XtalFreq::Mhz26), 26);
    }

    #[test]
    fn apll_coefficients_register_field_widths() {
        // o_div is a 5-bit field, sdm2 is 6-bit.
        for xtal in [XtalFreq::Mhz26, XtalFreq::Mhz32, XtalFreq::Mhz40] {
            let c = ApllCoefficients::for_xtal(xtal);
            assert!(
                c.o_div < 32,
                "o_div for {:?} = {} doesn't fit 5 bits",
                xtal,
                c.o_div
            );
            assert!(
                c.sdm2 < 64,
                "sdm2 for {:?} = {} doesn't fit 6 bits",
                xtal,
                c.sdm2
            );
        }
    }
}
