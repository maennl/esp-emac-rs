// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! ESP32-specific extension register definitions.
//!
//! Clock configuration, RMII/MII mode selection, GPIO routing,
//! and power management.
//! Base address: `0x3FF6_9800`.

#![allow(dead_code)]

// =============================================================================
// Base Address
// =============================================================================

/// Extension register block base address (ESP32).
pub const BASE: usize = 0x3FF6_9800;

// =============================================================================
// DPORT clock enable (system-level)
// =============================================================================

/// DPORT WiFi clock enable register (contains EMAC clock bit).
///
/// DPORT base = `0x3FF0_0000`, WIFI_CLK_EN offset = `0x0CC`.
pub const DPORT_WIFI_CLK_EN_REG: usize = 0x3FF0_00CC;

/// EMAC clock enable bit in `DPORT_WIFI_CLK_EN_REG`.
pub const DPORT_WIFI_CLK_EMAC_EN: u32 = 1 << 14;

/// DPORT core-reset register — pulsing [`DPORT_EMAC_RST`] here releases the
/// EMAC peripheral from reset. DPORT base = `0x3FF0_0000`, offset `0x0D0`.
///
/// Distinct from, and required *in addition to*,
/// [`DPORT_WIFI_CLK_EN_REG`]/[`DPORT_WIFI_CLK_EMAC_EN`]: the clock-enable
/// bit ungates the APB clock to the EMAC block, but the block's internal
/// registers stay held in reset (all reads/writes silently return/discard
/// a fixed value, as if unclocked) until this reset line is pulsed —
/// ESP-IDF's `emac_ll_reset_register()` (`emac_ll.h`) does exactly this
/// right after `emac_ll_enable_bus_clock()`, before any further EMAC
/// register access. Omitting it was observed on real hardware to produce
/// exactly that symptom: every EMAC/MDIO register (including PHY reads
/// relayed through GMACMIIDATA) reading back as a fixed `0`, with no bus
/// timeout — because the "poll until a status bit clears" pattern used
/// throughout this driver (DMA software reset, MDIO busy-bit, and every
/// downstream `PhyDriver::init` reset poll) trivially "succeeds" against
/// an always-zero register.
pub const DPORT_CORE_RST_EN_REG: usize = 0x3FF0_00D0;

/// EMAC reset bit in `DPORT_CORE_RST_EN_REG` (bit 7).
pub const DPORT_EMAC_RST: u32 = 1 << 7;

// =============================================================================
// IO_MUX
// =============================================================================

/// IO_MUX base address (ESP32). Re-exported here for the cross-module
/// consistency test in `crate::clock`; the bit-field constants live in
/// `crate::regs::gpio` where the GPIO Matrix routing actually uses them.
pub const IO_MUX_BASE: usize = 0x3FF4_9000;

// =============================================================================
// Register Offsets
// =============================================================================

/// Clock output configuration register.
pub const EX_CLKOUT_CONF: usize = 0x00;
/// Oscillator clock configuration register.
pub const EX_OSCCLK_CONF: usize = 0x04;
/// Clock control register.
pub const EX_CLK_CTRL: usize = 0x08;
/// PHY interface configuration register.
pub const EX_PHYINF_CONF: usize = 0x0C;
/// Power down select register.
pub const EX_PD_SEL: usize = 0x10;

// =============================================================================
// EX_CLKOUT_CONF bits (@ 0x00)
// =============================================================================

/// Bit-field constants for the Clock Output Configuration Register.
pub mod clkout_conf {
    /// Clock output divider number mask (bits 3:0).
    pub const DIV_NUM_MASK: u32 = 0x0F;
    /// Clock output half-period divider shift (bits 7:4).
    pub const H_DIV_NUM_SHIFT: u32 = 4;
    /// Clock output half-period divider mask.
    pub const H_DIV_NUM_MASK: u32 = 0x0F << 4;
    /// Delay number shift (bits 9:8).
    pub const DLY_NUM_SHIFT: u32 = 8;
    /// Delay number mask.
    pub const DLY_NUM_MASK: u32 = 0x03 << 8;
}

// =============================================================================
// EX_OSCCLK_CONF bits (@ 0x04)
// =============================================================================

/// Bit-field constants for the Oscillator Clock Configuration Register.
pub mod oscclk_conf {
    /// 10 Mbps divider number mask (bits 5:0).
    pub const DIV_NUM_10M_MASK: u32 = 0x3F;
    /// 10 Mbps half-period divider shift (bits 11:6).
    pub const H_DIV_NUM_10M_SHIFT: u32 = 6;
    /// 100 Mbps divider number shift (bits 17:12).
    pub const DIV_NUM_100M_SHIFT: u32 = 12;
    /// 100 Mbps half-period divider shift (bits 23:18).
    pub const H_DIV_NUM_100M_SHIFT: u32 = 18;
    /// Clock source select (bit 24): 0 = internal, 1 = external.
    pub const CLK_SEL: u32 = 1 << 24;
}

// =============================================================================
// EX_CLK_CTRL bits (@ 0x08)
// =============================================================================

/// Bit-field constants for the Clock Control Register.
pub mod clk_ctrl {
    /// External clock enable (bit 0) — enable external 50 MHz input.
    pub const EXT_EN: u32 = 1 << 0;
    /// Internal clock enable (bit 1) — enable internal APLL clock.
    pub const INT_EN: u32 = 1 << 1;
    /// RX 125 MHz clock enable (bit 2) — for gigabit mode.
    pub const RX_125_CLK_EN: u32 = 1 << 2;
    /// MII TX clock enable (bit 3).
    pub const MII_CLK_TX_EN: u32 = 1 << 3;
    /// MII RX clock enable (bit 4).
    pub const MII_CLK_RX_EN: u32 = 1 << 4;
    /// Main clock enable (bit 5).
    pub const CLK_EN: u32 = 1 << 5;
}

// =============================================================================
// EX_PHYINF_CONF bits (@ 0x0C)
// =============================================================================

/// Bit-field constants for the PHY Interface Configuration Register.
pub mod phyinf_conf {
    /// PHY interface select shift (3-bit field at bits 15:13).
    pub const PHY_INTF_SEL_SHIFT: u32 = 13;
    /// PHY interface select mask.
    pub const PHY_INTF_SEL_MASK: u32 = 0x07 << 13;
    /// PHY interface value for MII mode.
    pub const PHY_INTF_MII: u32 = 0;
    /// PHY interface value for RMII mode.
    pub const PHY_INTF_RMII: u32 = 4;
    /// SBD flow control enable (bit 2).
    pub const SBD_FLOWCTRL: u32 = 1 << 2;
    /// Core PHY address shift (5-bit field at bits 7:3).
    pub const CORE_PHY_ADDR_SHIFT: u32 = 3;
    /// Core PHY address mask.
    pub const CORE_PHY_ADDR_MASK: u32 = 0x1F << 3;
}

// =============================================================================
// EX_PD_SEL bits (@ 0x10)
// =============================================================================

/// Bit-field constants for the Power Down Select Register.
pub mod pd_sel {
    /// RAM power down enable mask (bits 1:0).
    pub const RAM_PD_EN_MASK: u32 = 0x03;
}

// =============================================================================
// Register access helpers
// =============================================================================

/// Read an EXT register at `offset` from BASE.
///
/// # Safety
/// Caller must ensure the EMAC peripheral clock is enabled and
/// `offset` is a valid register offset within this block.
#[inline(always)]
pub unsafe fn read(offset: usize) -> u32 {
    // SAFETY: caller guarantees address validity.
    core::ptr::read_volatile((BASE + offset) as *const u32)
}

/// Write an EXT register at `offset` from BASE.
///
/// # Safety
/// Caller must ensure the EMAC peripheral clock is enabled and
/// `offset` is a valid register offset within this block.
#[inline(always)]
pub unsafe fn write(offset: usize, val: u32) {
    // SAFETY: caller guarantees address validity.
    core::ptr::write_volatile((BASE + offset) as *mut u32, val);
}

/// Read-modify-write: set bits in an EXT register.
///
/// # Safety
/// Same requirements as [`read`] and [`write`].
#[inline(always)]
pub unsafe fn set_bits(offset: usize, bits: u32) {
    let val = read(offset);
    write(offset, val | bits);
}

/// Read-modify-write: clear bits in an EXT register.
///
/// # Safety
/// Same requirements as [`read`] and [`write`].
#[inline(always)]
pub unsafe fn clear_bits(offset: usize, bits: u32) {
    let val = read(offset);
    write(offset, val & !bits);
}

// =============================================================================
// Composite operations (formerly ph_esp32_mac::unsafe_registers::ExtRegs)
// =============================================================================

/// Enable the EMAC peripheral clock through the DPORT block.
///
/// MUST precede any access to the EMAC register blocks themselves —
/// `regs::mac` (`EMAC_MAC` base), `regs::dma` (`EMAC_DMA` base), and
/// the EMAC extension registers in this module above the `Composite
/// operations` section (`EX_CLK_CTRL`, `EX_PHYINF_CONF`, `EX_PD_SEL`,
/// …). Reads from those return garbage and writes are silently
/// dropped while the peripheral is unclocked.
///
/// Bring-up steps that touch *other* always-on blocks (RTC analog +
/// ROM I2C for APLL programming, IO_MUX, GPIO Matrix routing) work
/// before this call — they don't depend on the EMAC peripheral clock.
/// See `Emac::init` for the canonical ordering.
///
/// Also pulses [`DPORT_EMAC_RST`] (`DPORT_CORE_RST_EN_REG`) to release the
/// peripheral from reset — see that constant's doc comment for why this
/// is a separate, mandatory step from the clock-enable bit alone (mirrors
/// ESP-IDF's `emac_ll_enable_bus_clock()` + `emac_ll_reset_register()`
/// pair in `emac_ll.h`, always called back-to-back).
#[inline(always)]
pub fn enable_peripheral_clock() {
    // SAFETY: DPORT_WIFI_CLK_EN_REG and DPORT_CORE_RST_EN_REG are
    // known-valid 32-bit registers.
    unsafe {
        let cur = core::ptr::read_volatile(DPORT_WIFI_CLK_EN_REG as *const u32);
        core::ptr::write_volatile(
            DPORT_WIFI_CLK_EN_REG as *mut u32,
            cur | DPORT_WIFI_CLK_EMAC_EN,
        );

        // Pulse the EMAC reset line: assert, then immediately deassert.
        // `DPORT_CORE_RST_EN_REG` is a plain reset-enable register — other
        // bits are always 0 here (we own this narrow reset pulse and
        // nothing else touches this register concurrently), so a direct
        // write (not read-modify-write) matches ESP-IDF's own sequence.
        core::ptr::write_volatile(DPORT_CORE_RST_EN_REG as *mut u32, DPORT_EMAC_RST);
        core::ptr::write_volatile(DPORT_CORE_RST_EN_REG as *mut u32, 0);
    }
}

/// Enable the EMAC extension clocks (MII RX / MII TX / EMAC clock).
/// `enable_peripheral_clock()` must already have been called.
#[inline(always)]
pub fn enable_clocks() {
    // SAFETY: EX_CLK_CTRL is a known-valid 32-bit register inside the EXT block.
    unsafe {
        set_bits(
            EX_CLK_CTRL,
            clk_ctrl::MII_CLK_RX_EN | clk_ctrl::MII_CLK_TX_EN | clk_ctrl::CLK_EN,
        );
    }
}

/// Switch the PHY interface to RMII (`phy_intf_sel = 4`).
#[inline(always)]
pub fn set_rmii_mode() {
    // SAFETY: EX_PHYINF_CONF is a known-valid 32-bit register inside the EXT block.
    unsafe {
        let cur = read(EX_PHYINF_CONF);
        let new_val = (cur & !phyinf_conf::PHY_INTF_SEL_MASK)
            | (phyinf_conf::PHY_INTF_RMII << phyinf_conf::PHY_INTF_SEL_SHIFT);
        write(EX_PHYINF_CONF, new_val);
    }
}

/// Configure the EMAC clock for an external 50 MHz oscillator on GPIO0:
/// `EX_CLK_CTRL.ext_en = 1, int_en = 0`; `EX_OSCCLK_CONF.clk_sel = 1`.
#[inline(always)]
pub fn set_rmii_clock_external() {
    // SAFETY: both registers are known-valid 32-bit registers inside the EXT block.
    unsafe {
        let ctrl = read(EX_CLK_CTRL);
        write(EX_CLK_CTRL, (ctrl | clk_ctrl::EXT_EN) & !clk_ctrl::INT_EN);
        let osc = read(EX_OSCCLK_CONF);
        write(EX_OSCCLK_CONF, osc | oscclk_conf::CLK_SEL);
    }
}

/// Configure the EMAC clock for the internal APLL source:
/// `EX_CLK_CTRL.int_en = 1, ext_en = 0`; `EX_OSCCLK_CONF.clk_sel = 0`;
/// clear `EX_CLKOUT_CONF.div_num` and `h_div_num`.
#[inline(always)]
pub fn set_rmii_clock_internal() {
    // SAFETY: all three registers are known-valid 32-bit registers inside the EXT block.
    unsafe {
        let ctrl = read(EX_CLK_CTRL);
        write(EX_CLK_CTRL, (ctrl | clk_ctrl::INT_EN) & !clk_ctrl::EXT_EN);
        let osc = read(EX_OSCCLK_CONF);
        write(EX_OSCCLK_CONF, osc & !oscclk_conf::CLK_SEL);
        let clkout = read(EX_CLKOUT_CONF);
        write(
            EX_CLKOUT_CONF,
            clkout & !(clkout_conf::DIV_NUM_MASK | clkout_conf::H_DIV_NUM_MASK),
        );
    }
}

/// Power up the EMAC's internal RAM (`EX_PD_SEL.ram_pd = 0`).
#[inline(always)]
pub fn power_up_ram() {
    // SAFETY: EX_PD_SEL is a known-valid 32-bit register inside the EXT block.
    unsafe {
        let cur = read(EX_PD_SEL);
        write(EX_PD_SEL, cur & !pd_sel::RAM_PD_EN_MASK);
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base_address() {
        assert_eq!(BASE, 0x3FF6_9800);
    }

    #[test]
    fn register_offsets_within_block() {
        // EXT register block fits within 0x100 bytes
        let offsets = [
            EX_CLKOUT_CONF,
            EX_OSCCLK_CONF,
            EX_CLK_CTRL,
            EX_PHYINF_CONF,
            EX_PD_SEL,
        ];
        for off in offsets {
            assert!(off < 0x100, "offset {:#x} exceeds EXT block size", off);
        }
    }

    #[test]
    fn clk_ctrl_bits_no_overlap() {
        let bits = [
            clk_ctrl::EXT_EN,
            clk_ctrl::INT_EN,
            clk_ctrl::RX_125_CLK_EN,
            clk_ctrl::MII_CLK_TX_EN,
            clk_ctrl::MII_CLK_RX_EN,
            clk_ctrl::CLK_EN,
        ];
        for i in 0..bits.len() {
            for j in (i + 1)..bits.len() {
                assert_eq!(
                    bits[i] & bits[j],
                    0,
                    "clk_ctrl bits {:#x} and {:#x} overlap",
                    bits[i],
                    bits[j]
                );
            }
        }
    }

    #[test]
    fn phyinf_rmii_value_fits_mask() {
        let rmii_val = phyinf_conf::PHY_INTF_RMII << phyinf_conf::PHY_INTF_SEL_SHIFT;
        assert_eq!(rmii_val & phyinf_conf::PHY_INTF_SEL_MASK, rmii_val);
    }

    #[test]
    fn phyinf_mii_value_is_zero() {
        assert_eq!(phyinf_conf::PHY_INTF_MII, 0);
    }

    #[test]
    fn dport_emac_clock_bit_position() {
        // Bit 14 in the DPORT WIFI_CLK_EN register
        assert_eq!(DPORT_WIFI_CLK_EMAC_EN, 1 << 14);
        assert_eq!(DPORT_WIFI_CLK_EMAC_EN, 0x4000);
    }

    #[test]
    fn base_addresses_in_order() {
        // DMA < EXT < MAC
        assert!(super::super::dma::BASE < BASE);
        assert!(BASE < super::super::mac::BASE);
    }
}
