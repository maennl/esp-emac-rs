// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! EMAC configuration types.

/// EMAC configuration.
#[derive(Debug, Clone)]
pub struct EmacConfig {
    /// RMII clock source.
    pub clock: RmiiClockConfig,
    /// RMII management pins (MDC/MDIO).
    pub pins: RmiiPins,
}

/// RMII reference clock configuration.
///
/// The 50 MHz RMII clock can be generated internally (ESP32 APLL) or
/// supplied externally from a PHY-driven oscillator. Mode selection is
/// hardware-specific and `Emac::init` rejects mismatched GPIO choices
/// with [`crate::EmacError::InvalidConfig`] — see each variant's docs
/// for which pads are valid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RmiiClockConfig {
    /// ESP32 APLL generates 50 MHz and drives it out of the chip.
    ///
    /// Valid GPIO choices: [`ClkGpio::Gpio16`] (`EMAC_CLK_OUT`, 0°) or
    /// [`ClkGpio::Gpio17`] (`EMAC_CLK_OUT_180`, 180° — the LAN8720A
    /// reference design preference). [`ClkGpio::Gpio0`] is **invalid**
    /// for this mode: GPIO0 function 5 is `EMAC_TX_CLK`, an input pad.
    ///
    /// `xtal` selects the SDM coefficients for APLL programming and
    /// MUST match the actual on-board crystal — there is no detection
    /// at runtime, getting it wrong silently produces a wrong-frequency
    /// REF_CLK and the link will not come up.
    ///
    /// Coexistence note: ESP32 errata CLK-3.22 — the APLL clock signal
    /// emitted on the GPIO pad is corrupted by on-chip RF noise during
    /// WiFi/BT transmission. This mode is unsafe with active radio;
    /// boards needing Ethernet + WiFi should use [`Self::External`].
    InternalApll {
        /// GPIO for clock output. Must be `Gpio16` or `Gpio17`.
        gpio: ClkGpio,
        /// On-board crystal frequency. APLL SDM coefficients are
        /// chosen accordingly to land on a 50 MHz RMII reference clock.
        xtal: XtalFreq,
    },
    /// External 50 MHz clock fed in from a PHY crystal or oscillator.
    ///
    /// Valid GPIO choice: [`ClkGpio::Gpio0`] only — that is the only
    /// pad whose function 5 (`EMAC_TX_CLK`) is an input. `Gpio16` /
    /// `Gpio17` are **invalid** here.
    ///
    /// Required for Ethernet + WiFi coexistence (immune to the
    /// CLK-3.22 errata since the clock never leaves the PHY domain).
    External {
        /// GPIO for clock input. Must be `Gpio0`.
        gpio: ClkGpio,
    },
    /// ESP32 APLL generates 50 MHz and drives it out on **GPIO0** via the
    /// generic "clock output" mux (`PIN_CTRL.CLK_OUT1` → `CLKOUT_CHANNEL_1`
    /// → GPIO0 IO_MUX function 1), instead of the dedicated EMAC clock-out
    /// pins used by [`Self::InternalApll`].
    ///
    /// GPIO0's EMAC IO_MUX function 5 (`EMAC_TX_CLK`) is hard-wired as an
    /// **input** inside the EMAC peripheral, so it cannot carry an
    /// APLL-driven output the way GPIO16/17 do. ESP-IDF instead routes the
    /// APLL signal through the SoC's independent clock-output mux
    /// (`esp_clock_output`/`CLKOUT_SIG_APLL`), whose channel 1 is fixed to
    /// GPIO0 (IO_MUX function 1, `FUNC_GPIO0_CLK_OUT1` — *not* function 5).
    /// The EMAC-internal clock-source-select bit is programmed identically
    /// to [`Self::InternalApll`] ("internal"/RMII-output) — only the pad
    /// routing differs.
    ///
    /// ESP-IDF marks this mode "(Experimental!)" — it works on boards that
    /// wire GPIO0 to the PHY's REF_CLK pin instead of GPIO16/17 (e.g. no
    /// separate crystal on the PHY), but is documented as potentially
    /// unreliable with some PHY chips. Also note GPIO0 is a boot-strapping
    /// pin — this mode only drives the pad after `Emac::init` runs, well
    /// after reset/boot-mode sampling has completed.
    InternalApllClkOutGpio0 {
        /// On-board crystal frequency, see [`XtalFreq`].
        xtal: XtalFreq,
    },
}

/// On-board crystal frequency in MHz, used to pick APLL SDM coefficients
/// for a 50 MHz RMII reference-clock output.
///
/// ESP32 SoC accepts crystals at 26, 32, or 40 MHz. The vast majority
/// of modules ship with 40 MHz (`ESP32-WROOM-32`, `ESP32-WROVER`,
/// `ESP32-MINI-1` and most reference designs). 26 MHz appears in older
/// QFN-only designs; 32 MHz is rare custom-design territory. The crate
/// only provides coefficients for these three values — for any other
/// crystal you will need to extend [`crate::clock::ApllCoefficients`]
/// and submit a patch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum XtalFreq {
    /// 26 MHz — legacy ESP32 modules, certain QFN bare-chip designs.
    Mhz26,
    /// 32 MHz — rare custom designs.
    Mhz32,
    /// 40 MHz — ESP32-WROOM, ESP32-WROVER, ESP32-MINI, most modern boards.
    Mhz40,
}

impl XtalFreq {
    /// Frequency in MHz as a `u32`. Useful for logging / diagnostics.
    pub const fn mhz(self) -> u32 {
        match self {
            Self::Mhz26 => 26,
            Self::Mhz32 => 32,
            Self::Mhz40 => 40,
        }
    }
}

/// GPIO pins that can carry the EMAC RMII reference clock on ESP32.
///
/// Direction is fixed by the IO_MUX function 5 wiring:
///
/// - [`Self::Gpio0`] — `EMAC_TX_CLK`, input only — pair with
///   [`RmiiClockConfig::External`].
/// - [`Self::Gpio16`] — `EMAC_CLK_OUT` (0°), output only — pair with
///   [`RmiiClockConfig::InternalApll`].
/// - [`Self::Gpio17`] — `EMAC_CLK_OUT_180` (180°), output only — pair
///   with [`RmiiClockConfig::InternalApll`]. Most common on LAN8720A.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClkGpio {
    /// GPIO0 — `EMAC_TX_CLK` input. Also a boot-strapping pin, take
    /// care that the external oscillator does not violate boot timing.
    Gpio0,
    /// GPIO16 — `EMAC_CLK_OUT` (0° phase).
    Gpio16,
    /// GPIO17 — `EMAC_CLK_OUT_180` (180° phase, most common for LAN8720A).
    Gpio17,
}

impl ClkGpio {
    /// Get the GPIO number.
    pub const fn gpio_num(self) -> u8 {
        match self {
            ClkGpio::Gpio0 => 0,
            ClkGpio::Gpio16 => 16,
            ClkGpio::Gpio17 => 17,
        }
    }
}

/// RMII management pin configuration.
///
/// Data pins (TXD0/1, RXD0/1, TX_EN, CRS_DV) are fixed on ESP32 via IO_MUX
/// and cannot be remapped. Only MDC/MDIO are routed via GPIO Matrix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RmiiPins {
    /// MDC (Management Data Clock) — routed via GPIO Matrix, any GPIO.
    pub mdc: u8,
    /// MDIO (Management Data I/O) — routed via GPIO Matrix, any GPIO.
    pub mdio: u8,
}

impl Default for RmiiPins {
    /// Default: MDC=GPIO23, MDIO=GPIO18 (most common ESP32 Ethernet boards).
    fn default() -> Self {
        Self { mdc: 23, mdio: 18 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clk_gpio_num() {
        assert_eq!(ClkGpio::Gpio0.gpio_num(), 0);
        assert_eq!(ClkGpio::Gpio16.gpio_num(), 16);
        assert_eq!(ClkGpio::Gpio17.gpio_num(), 17);
    }

    #[test]
    fn rmii_pins_default() {
        let pins = RmiiPins::default();
        assert_eq!(pins.mdc, 23);
        assert_eq!(pins.mdio, 18);
    }

    #[test]
    fn rmii_clock_config_equality() {
        let a = RmiiClockConfig::InternalApll {
            gpio: ClkGpio::Gpio17,
            xtal: XtalFreq::Mhz40,
        };
        let b = RmiiClockConfig::InternalApll {
            gpio: ClkGpio::Gpio17,
            xtal: XtalFreq::Mhz40,
        };
        let c = RmiiClockConfig::External {
            gpio: ClkGpio::Gpio0,
        };
        let d = RmiiClockConfig::InternalApll {
            gpio: ClkGpio::Gpio17,
            xtal: XtalFreq::Mhz26,
        };
        assert_eq!(a, b);
        assert_ne!(a, c);
        // Different XTAL — different config.
        assert_ne!(a, d);
    }

    #[test]
    fn xtal_freq_mhz_values() {
        assert_eq!(XtalFreq::Mhz26.mhz(), 26);
        assert_eq!(XtalFreq::Mhz32.mhz(), 32);
        assert_eq!(XtalFreq::Mhz40.mhz(), 40);
    }

    #[test]
    fn emac_config_clone() {
        let config = EmacConfig {
            clock: RmiiClockConfig::InternalApll {
                gpio: ClkGpio::Gpio17,
                xtal: XtalFreq::Mhz40,
            },
            pins: RmiiPins::default(),
        };
        let cloned = config.clone();
        assert_eq!(cloned.pins.mdc, 23);
    }
}
