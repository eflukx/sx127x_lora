#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]
#![crate_name = "sx127x_lora"]

//! # sx127x_lora
//!  A platform-agnostic driver for Semtech SX1276/77/78/79 based boards. It supports any device that
//! implements the `embedded-hal` traits. Devices are connected over SPI and require an extra GPIO pin for
//! RESET. This cate works with any Semtech based board including:
//! * Modtronix inAir4, inAir9, and inAir9B
//! * HopeRF RFM95W, RFM96W, and RFM98W
//! # Examples
//! ## Raspberry Pi Basic Send
//! Utilizes a Raspberry Pi to send a message. The example utilizes the `linux_embedded_hal` crate.
//! ```no_run
//! #![feature(extern_crate_item_prelude)]
//! extern crate sx127x_lora;
//! extern crate linux_embedded_hal as hal;
//!
//! use hal::spidev::{self, SpidevOptions};
//! use hal::{Pin, Spidev};
//! use hal::sysfs_gpio::Direction;
//! use hal::Delay;

//! const LORA_CS_PIN: u64 = 8;
//! const LORA_RESET_PIN: u64 = 21;
//! const FREQUENCY: i64 = 868_100_000;
//!
//! fn main(){
//!
//!     let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
//!     let options = SpidevOptions::new()
//!         .bits_per_word(8)
//!         .max_speed_hz(20_000)
//!         .mode(spidev::SPI_MODE_0)
//!         .build();
//!     spi.configure(&options).unwrap();
//!
//!     let cs = Pin::new(LORA_CS_PIN);
//!     cs.export().unwrap();
//!     cs.set_direction(Direction::Out).unwrap();
//!
//!     let reset = Pin::new(LORA_RESET_PIN);
//!     reset.export().unwrap();
//!     reset.set_direction(Direction::Out).unwrap();
//!
//!     let mut lora = sx127x_lora::LoRa::new(
//!         spi, cs, reset,  FREQUENCY, Delay)
//!         .expect("Failed to communicate with radio module!");
//!
//!     lora.set_tx_power(14,1); //Using PA_BOOST. See your board for correct pin.
//!
//!     let message = "Hello, world!";
//!     let mut buffer = [0;255];
//!     for (i,c) in message.chars().enumerate() {
//!         buffer[i] = c as u8;
//!     }
//!
//!     let transmit = lora.transmit_payload(buffer,message.len());
//!     match transmit {
//!         Ok(packet_size) => println!("Sent packet with size: {}", packet_size),
//!         Err(()) => println!("Error"),
//!     }
//! }
//! ```
//! ## STM32F429 Blocking Receive
//! Utilizes a STM32F429 to receive data using the blocking `poll_irq(timeout)` function. It prints
//! the received packet back out over semihosting. The example utilizes the `stm32f429_hal`, `cortex_m`,
//! and `panic_semihosting` crates.
//! ```no_run
//! #![no_std]
//! #![no_main]
//!
//! extern crate sx127x_lora;
//! extern crate stm32f429_hal as hal;
//! extern crate cortex_m;
//! extern crate panic_semihosting;
//!
//! use sx127x_lora::MODE;
//! use cortex_m_semihosting::*;
//! use hal::gpio::GpioExt;
//! use hal::flash::FlashExt;
//! use hal::rcc::RccExt;
//! use hal::time::MegaHertz;
//! use hal::spi::Spi;
//! use hal::delay::Delay;
//!
//! const FREQUENCY: i64 = 915;
//!
//! #[entry]
//! fn main() -> !{
//!     let cp = cortex_m::Peripherals::take().unwrap();
//!     let p = hal::stm32f429::Peripherals::take().unwrap();
//!
//!     let mut rcc = p.RCC.constrain();
//!     let mut flash = p.FLASH.constrain();
//!     let clocks = rcc
//!         .cfgr
//!         .sysclk(MegaHertz(64))
//!         .pclk1(MegaHertz(32))
//!         .freeze(&mut flash.acr);
//!
//!     let mut gpioa = p.GPIOA.split(&mut rcc.ahb1);
//!     let mut gpiod = p.GPIOD.split(&mut rcc.ahb1);
//!     let mut gpiof = p.GPIOF.split(&mut rcc.ahb1);
//!
//!     let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
//!     let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
//!     let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
//!     let reset = gpiof.pf13.into_push_pull_output(&mut gpiof.moder, &mut gpiof.otyper);
//!     let cs = gpiod.pd14.into_push_pull_output(&mut gpiod.moder, &mut gpiod.otyper);
//!
//!     let spi = Spi::spi1(
//!         p.SPI1,
//!         (sck, miso, mosi),
//!         MODE,
//!         MegaHertz(8),
//!         clocks,
//!         &mut rcc.apb2,
//!     );
//!
//!     let mut lora = sx127x_lora::LoRa::new(
//!         spi, cs, reset, FREQUENCY,
//!         Delay::new(cp.SYST, clocks)).unwrap();
//!
//!     loop {
//!         let poll = lora.poll_irq(Some(30)); //30 Second timeout
//!         match poll {
//!             Ok(size) =>{
//!                hprint!("with Payload: ");
//!                let buffer = lora.read_packet(); // Received buffer. NOTE: 255 bytes are always returned
//!                for i in 0..size{
//!                    hprint!("{}",buffer[i] as char).unwrap();
//!                }
//!                hprintln!();
//!             },
//!             Err(()) => hprintln!("Timeout").unwrap(),
//!         }
//!     }
//! }
//! ```
//! ## Interrupts
//! The crate currently polls the IRQ register on the radio to determine if a new packet has arrived. This
//! would be more efficient if instead an interrupt was connect the the module's DIO_0 pin. Once interrupt
//! support is available in `embedded-hal`, then this will be added. It is possible to implement this function on a
//! device-to-device basis by retrieving a packet with the `read_packet()` function.

use core::convert::TryFrom;

use bit_field::BitField;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::{Mode, Phase, Polarity};

mod register;
pub use register::{
    BwSetting, FskDataModulationShaping, FskRampUpRamDown, IrqFlags, PaConfig, RadioMode,
};
use register::{Register::*, IRQ};

use Error::*;

pub const FIFO_SIZE: usize = 256;

pub const SPI_MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

pub struct LoRa<SPI, CS, RESET> {
    spi: SPI,
    cs: CS,
    reset: RESET,
    freq_hz: u32,
    pub explicit_header: bool,
    pub mode: RadioMode,
}

#[derive(Debug)]
pub enum Error<SPI, CS, RESET> {
    Uninformative,
    RxTimeout,
    VersionMismatch(u8),
    CS(CS),
    Reset(RESET),
    SPI(SPI),
    Transmitting,
    IllegalBwSetting,
}

#[cfg(not(feature = "version_0x09"))]
const VERSION_CHECK: u8 = 0x12;

#[cfg(feature = "version_0x09")]
const VERSION_CHECK: u8 = 0x09;

/// Calculates the symbol time in µs when given SpreadFactor and BandWidth
pub fn calc_symbol_time_us(sf: u8, bw: BwSetting) -> u32 {
    let spow = 1 << sf as u32; // (2.pow(sf))
    (1_000_000 * (spow + 32)) / bw.as_hz() // +32 'magic' from the datasheet
}

pub fn calc_data_rate(sf: u8, bw: BwSetting, cr: u8) -> u32 {
    let sf = sf as u32;
    (sf * (bw.as_hz() / (1 << sf)) * 4) / cr as u32
}

impl<SPI, CS, RESET, E> LoRa<SPI, CS, RESET>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
    RESET: OutputPin,
{
    /// Builds and returns a new instance of the radio. Only one instance of the radio should exist at a time.
    /// This also preforms a hardware reset of the module and then puts it in standby.
    pub fn new(
        freq_hz: u32,
        spi: SPI,
        cs: CS,
        reset: RESET,
        delay: &mut dyn DelayMs<u8>,
    ) -> Result<Self, Error<E, CS::Error, RESET::Error>> {
        let mut sx127x = LoRa {
            spi,
            cs,
            reset,
            freq_hz,
            explicit_header: true,
            mode: RadioMode::Sleep,
        };

        sx127x.reset.set_low().map_err(Reset)?;
        delay.delay_ms(10);
        sx127x.reset.set_high().map_err(Reset)?;
        delay.delay_ms(10);

        let version = sx127x.get_hw_version()?;
        if version == VERSION_CHECK {
            sx127x.set_mode(RadioMode::Sleep)?;
            sx127x.set_frequency(sx127x.freq_hz)?;

            sx127x.write_register(RegFifoTxBaseAddr, 0)?;
            sx127x.write_register(RegFifoRxBaseAddr, 0)?;

            // let lna = sx127x.read_register(RegLna)?;
            // sx127x.write_register(RegLna, lna | 0x03)?;
            sx127x.write_register(RegLna, register::LNA_MAX_GAIN)?;

            sx127x.write_register(RegModemConfig3, 0x04)?;
            // sx127x.write_register(RegModemConfig3, 0x0C)?; // for sf11-12

            sx127x.write_register(RegSyncWord, 0x34)?; // LoRaWAN public sync word

            sx127x.set_mode(RadioMode::Stdby)?; // Need to go via Stdby, else ~2mA current draw.
            sx127x.set_mode(RadioMode::Sleep)?;

            Ok(sx127x)
        } else {
            Err(Error::VersionMismatch(version))
        }
    }

    pub fn free(mut self) -> Result<(SPI, CS, RESET), Error<E, CS::Error, RESET::Error>> {
        self.set_mode(RadioMode::Sleep)?;
        Ok((self.spi, self.cs, self.reset))
    }

    pub fn transmit_blocking(
        &mut self,
        payload: &[u8],
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        self.transmit_nb(payload)?;
        while self.is_transmitting()? {}
        Ok(())
    }

    pub fn transmit_nb(&mut self, payload: &[u8]) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        if self.is_transmitting()? {
            Err(Transmitting)
        } else {
            self.set_mode(RadioMode::Stdby)?;

            self.clear_irq_flags()?;
            self.write_register(RegIrqFlagsMask, IRQ::TxDone.as_byte() ^ 0xff)?;
            self.write_register(RegDioMapping1, 0x40 | 0x30 | 0xc0)?;

            self.write_register(RegFifoTxBaseAddr, 0)?;
            self.write_register(RegFifoAddrPtr, 0)?;
            self.write_register(RegPayloadLength, payload.len().min(255) as u8)?;

            for &byte in payload.iter().take(255) {
                self.write_register(RegFifo, byte)?;
            }

            self.set_mode(RadioMode::Tx)
        }
    }

    /// Blocking RX of a single packet. This function needs to be called *on time*, exactly when a packet is expected
    /// It returns the size of a packet if Rx was successful. An error is returned if the RX task timed out.
    pub fn polled_rx_single(
        &mut self,
        delay: &mut dyn DelayMs<u8>,
    ) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        self.write_register(RegFifoRxBaseAddr, 0)?; // start writing FIFO from the bottom, so we can use the full 256 bytes FIFO capacity
        self.clear_irq_flags()?;
        self.write_register(
            RegIrqFlagsMask,
            (IRQ::ValidHeader.as_byte() | IRQ::RxDone.as_byte()) ^ 0xff,
        )?;

        self.set_mode(RadioMode::RxSingle)?;

        let mut total_rx_timeout_ms = (FIFO_SIZE as u32 * 10 * 1000) / self.get_data_rate_bps()?; // fifo_size * 10; 10-bit-bytes to add some leeway/margin
        let mut header_timeout_ms = 175; // iets met symbolrate / preamble / header size

        // Wait/poll for valid header
        while header_timeout_ms > 0 && !self.read_irq_flags()?.valid_header() {
            header_timeout_ms -= 1;
            delay.delay_ms(1);
        }

        defmt::debug!("polled_rx_single: after wait for header:\nflags {}, header_timeout_left_ms {}ms, total_rx_timeout_ms: {}ms",
        defmt::Debug2Format(&self.read_irq_flags()?),
            header_timeout_ms,
            total_rx_timeout_ms,
        );

        // Yes we have received a valid header!
        let result = if self.read_irq_flags()?.valid_header() {
            while total_rx_timeout_ms > 0 && !self.read_irq_flags()?.rx_done() {
                total_rx_timeout_ms -= 1;
                delay.delay_ms(1);
            }

            if self.read_irq_flags()?.rx_done() {
                self.print_fifo_info("in polled_rx_single")?;
                Ok(self.get_rx_packet_size()?)
            } else {
                Err(RxTimeout)
            }
        } else {
            Err(RxTimeout)
        };

        self.set_mode(RadioMode::Stdby)?;

        result
    }

    /// Blocks the current thread, returning the size of a packet if one is received or an error is the
    /// task timed out. The timeout can be supplied with None to make it poll indefinitely or
    /// with `Some(timeout_in_mill_seconds)`
    pub fn polled_rx(
        &mut self,
        timeout_ms: Option<i32>,
        delay: &mut dyn DelayMs<u8>,
    ) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        self.set_mode(RadioMode::RxContinuous)?;

        match timeout_ms {
            Some(timeout) => {
                let mut count = 0;
                let packet_ready = loop {
                    let packet_ready = self.read_irq_flags()?.rx_done(); // valid headeR: | irq_flags.get_bit(4);

                    if count >= timeout || packet_ready {
                        break packet_ready;
                    }
                    count += 1;
                    delay.delay_ms(1);
                };

                if packet_ready {
                    self.clear_irq_flags()?;
                    Ok(self.get_rx_packet_size()?)
                } else {
                    Err(Uninformative)
                }
            }
            None => {
                while !self.read_irq_flags()?.rx_done() {
                    delay.delay_ms(10);
                }
                self.clear_irq_flags()?;
                Ok(self.get_rx_packet_size()?)
            }
        }
    }

    /// Writes content in the given buffer and returns a slice into the buffer containing the
    /// This should only be called if there is a new packet ready to be read.
    pub fn read_rx_fifo_into<'a, const N: usize>(
        &mut self,
        buffer: &'a mut [u8; N],
    ) -> Result<&'a [u8], Error<E, CS::Error, RESET::Error>> {
        self.clear_irq_flags()?;
        self.print_fifo_info("read packet")?;
        let size = self.get_rx_packet_size()?;

        // Set read pointer to RxBase (start of reception) pointer in the FIFO
        let rx_base_ptr = self.read_register(RegFifoRxBaseAddr)?;
        self.write_register(RegFifoAddrPtr, rx_base_ptr)?;
        self.print_fifo_info("read packet")?;

        for i in 0..size {
            let byte = self.read_register(RegFifo)?;
            buffer[i as usize] = byte;
        }
        // self.write_register(RegFifoAddrPtr, 0)?;

        Ok(&buffer[..size as usize])
    }

    /// Returns the contents of the fifo as a [u8; 256]
    /// This should only be called if there is a new packet ready to be read.
    pub fn read_rx_fifo(&mut self) -> Result<[u8; 256], Error<E, CS::Error, RESET::Error>> {
        let mut buffer = [0 as u8; 256];
        self.read_rx_fifo_into(&mut buffer)?;
        Ok(buffer)
    }

    /// Returns size of a packet read into FIFO. This should only be calle if there is a new packet
    /// ready to be read.
    pub fn get_rx_packet_size(&mut self) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        self.read_register(RegRxNbBytes)
    }

    /// Returns true if the radio is currently transmitting a packet.
    pub fn is_transmitting(&mut self) -> Result<bool, Error<E, CS::Error, RESET::Error>> {
        let op_mode = self.read_register(RegOpMode)? & 0x7;

        if (op_mode == RadioMode::Tx.as_byte()) || (op_mode == RadioMode::FsTx.as_byte()) {
            Ok(true)
        } else {
            if self.read_irq_flags()?.rx_done() {
                self.write_register(RegIrqFlags, IRQ::TxDone.as_byte())?;
            }
            Ok(false)
        }
    }

    pub fn print_fifo_info(
        &mut self,
        prefix: &str,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let fifo_addr = self.read_register(RegFifoAddrPtr)?; // fifo host pointer
        let rx_current = self.read_register(RegFifoRxCurrentAddr)?; // fifo
        let rx_base = self.read_register(RegFifoRxBaseAddr)?; // fifo
        let tx_base = self.read_register(RegFifoTxBaseAddr)?; // Modem tx add

        defmt::error!(
            "\n\n SX1276 FIFO info!: ({})\nfifo_addr: {}, rx_current {}, rx_base {}, tx_base {}",
            prefix,
            fifo_addr,
            rx_current,
            rx_base,
            tx_base
        );
        Ok(())
    }

    /// Clears the radio's IRQ registers.
    pub fn clear_irq_flags(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        // let irq_flags = self.read_irq_flags()?;
        // self.write_register(RegIrqFlags, irq_flags.as_byte())
        self.write_register(RegIrqFlags, 0xff)
    }

    /// Sets the transmit power and pin. Levels can range from 0-14 when the output
    /// pin = 0(RFO), and form 0-20 when output pin = 1(PaBoost). Power is in dB.
    /// Default value is `17`.
    pub fn set_tx_power(
        &mut self,
        mut level: i32,
        output_pin: u8,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        if PaConfig::PaOutputRfoPin.as_byte() == output_pin {
            // RFO
            if level < 0 {
                level = 0;
            } else if level > 14 {
                level = 14;
            }
            self.write_register(RegPaConfig, (0x70 | level) as u8)
        } else {
            // PA BOOST
            if level > 17 {
                if level > 20 {
                    level = 20;
                }
                // subtract 3 from level, so 18 - 20 maps to 15 - 17
                level -= 3;

                // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
                self.write_register(RegPaDac, 0x87)?;
                self.set_ocp(140)?;
            } else {
                if level < 2 {
                    level = 2;
                }
                //Default value PA_HF/LF or +17dBm
                self.write_register(RegPaDac, 0x84)?;
                self.set_ocp(100)?;
            }
            level -= 2;
            self.write_register(RegPaConfig, PaConfig::PaBoost.as_byte() | level as u8)
        }
    }

    /// Sets the over current protection on the radio(mA).
    pub fn set_ocp(&mut self, ma: u8) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let mut ocp_trim: u8 = 27;

        if ma <= 120 {
            ocp_trim = (ma - 45) / 5;
        } else if ma <= 240 {
            ocp_trim = (ma + 30) / 10;
        }
        self.write_register(RegOcp, 0x20 | (0x1F & ocp_trim))
    }

    /// Sets the state of the radio. Default mode after initiation is `Standby`.
    pub fn set_mode(&mut self, mode: RadioMode) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        if self.explicit_header {
            self.set_explicit_header_mode()?;
        } else {
            self.set_implicit_header_mode()?;
        }

        self.write_register(
            RegOpMode,
            RadioMode::LongRangeMode.as_byte() | mode.as_byte(),
        )?;

        self.mode = mode;
        Ok(())
    }

    pub fn get_hw_version(&mut self) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        self.read_register(RegVersion)
    }

    /// Sets the frequency of the radio. Values are in Hertz.
    ///
    pub fn set_frequency(
        &mut self,
        freq_hz: u32,
    ) -> Result<u32, Error<E, CS::Error, RESET::Error>> {
        self.freq_hz = freq_hz;
        let frf = ((freq_hz as i64) << 19) / 32_000_000;
        self.write_register(RegFrfMsb, ((frf >> 16) & 0xff) as u8)?;
        self.write_register(RegFrfMid, ((frf >> 8) & 0xff) as u8)?;
        self.write_register(RegFrfLsb, ((frf >> 0) & 0xff) as u8)?;

        let actual = (frf * 32_000_000) >> 19;
        Ok(actual as u32)
    }

    /// Gets observed frequency error in Hertz
    pub fn get_freq_error_hz(&mut self) -> Result<i32, Error<E, CS::Error, RESET::Error>> {
        let fei = self.get_fei_value()? as i64;
        let bw = self.get_signal_bandwidth()?.as_hz() as i64;

        // See page 4.1.5 (pag. 37) of Semtech sx1276 datasheet (rev.7)
        let err_hz = (fei * (1 << 24) * bw) / (32_000_000 * 500_000);

        Ok(err_hz as i32)
    }

    /// Gets raw 20bit frequency error (FEI) value
    pub fn get_fei_value(&mut self) -> Result<i32, Error<E, CS::Error, RESET::Error>> {
        let lsb = self.read_register(RegFreqErrorLsb)?;
        let mid = self.read_register(RegFreqErrorMid)?;
        let msb = self.read_register(RegFreqErrorMsb)?;

        let n_msb: u8 = msb << 4 | mid >> 4;
        let n_mid: u8 = mid << 4 | lsb >> 4;
        let n_lsb: u8 = lsb << 4;
        let fei_value = i32::from_be_bytes([n_msb, n_mid, n_lsb, 0]) >> 12; // 20bit 2's complement, expect bit shift to sign-extend

        Ok(fei_value)
    }

    /// Sets the radio to use an explicit header. Default state is `ON`.
    fn set_explicit_header_mode(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let reg_modem_config_1 = self.read_register(RegModemConfig1)?;
        self.write_register(RegModemConfig1, reg_modem_config_1 & 0xfe)?;
        self.explicit_header = true;
        Ok(())
    }

    /// Sets the radio to use an implicit header. Default state is `OFF`.
    fn set_implicit_header_mode(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let reg_modem_config_1 = self.read_register(RegModemConfig1)?;
        self.write_register(RegModemConfig1, reg_modem_config_1 & 0x01)?;
        self.explicit_header = false;
        Ok(())
    }

    /// Sets the spreading factor of the radio. Supported values are between 6 and 12.
    /// If a spreading factor of 6 is set, implicit header mode must be used to transmit
    /// and receive packets. Default value is `7`.
    pub fn set_spread_factor(
        &mut self,
        mut sf: u8,
    ) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        if sf < 6 {
            sf = 6;
        } else if sf > 12 {
            sf = 12;
        }

        if sf == 6 {
            self.write_register(RegDetectionOptimize, 0xc5)?;
            self.write_register(RegDetectionThreshold, 0x0c)?;
        } else {
            self.write_register(RegDetectionOptimize, 0xc3)?;
            self.write_register(RegDetectionThreshold, 0x0a)?;
        }
        let modem_config_2 = self.read_register(RegModemConfig2)?;
        self.write_register(
            RegModemConfig2,
            (modem_config_2 & 0x0f) | ((sf << 4) & 0xf0),
        )?;
        self.set_ldo_flag()?;
        Ok(sf) // actual SF
    }

    /// Sets the signal bandwidth of the radio. Supported values are: `7800 Hz`, `10400 Hz`,
    /// `15600 Hz`, `20800 Hz`, `31250 Hz`,`41700 Hz` ,`62500 Hz`,`125000 Hz` and `250000 Hz`
    /// Default value is `125000 Hz`
    /// See p. 4 of SX1276_77_8_ErrataNote_1.1_STD.pdf for Errata implemetation
    pub fn set_signal_bandwidth(
        &mut self,
        bw: BwSetting,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        if bw == BwSetting::Bw500kHz {
            if self.freq_hz < 525_000_000 {
                self.write_register(RegHighBWOptimize1, 0x02)?;
                self.write_register(RegHighBWOptimize2, 0x7f)?;
            } else {
                self.write_register(RegHighBWOptimize1, 0x02)?;
                self.write_register(RegHighBWOptimize2, 0x64)?;
            }
        } else {
            self.write_register(RegHighBWOptimize1, 0x03)?;
            self.write_register(RegHighBWOptimize2, 0x65)?;
        }

        let modem_config_1 = self.read_register(RegModemConfig1)?;
        self.write_register(RegModemConfig1, (modem_config_1 & 0x0f) | ((bw as u8) << 4))?;
        self.set_ldo_flag()?;
        Ok(())
    }

    /// Sets the coding rate of the radio with the numerator fixed at 4. Supported values
    /// are between `5` and `8`, these correspond to coding rates of `4/5` and `4/8`.
    /// Default value is `5`.
    pub fn set_coding_rate_4(
        &mut self,
        mut denominator: u8,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        if denominator < 5 {
            denominator = 5;
        } else if denominator > 8 {
            denominator = 8;
        }
        let cr = denominator - 4;
        let modem_config_1 = self.read_register(RegModemConfig1)?;
        self.write_register(RegModemConfig1, (modem_config_1 & 0xf1) | (cr << 1))
    }

    /// Reads the coding rate of the radio with the numerator fixed at 4.
    pub fn get_coding_rate_4(&mut self) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        let modem_config_1 = self.read_register(RegModemConfig1)?;
        Ok(((modem_config_1 & 0x0f) >> 1) + 4)
    }

    /// Sets the programmable preamble length of the radio (in symbols). Values are between 6 and 65535.
    /// Default value is `8`, total 'real' preamble time is Tsym * (preamble_length + 4.25)
    pub fn set_preamble_length(
        &mut self,
        length: u16,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        self.write_register(RegPreambleMsb, (length >> 8) as u8)?;
        self.write_register(RegPreambleLsb, (length & 0xff) as u8)
    }

    /// Gets the programmable preamble length (in symbols).
    pub fn get_preamble_length(&mut self) -> Result<u16, Error<E, CS::Error, RESET::Error>> {
        let msb = self.read_register(RegPreambleMsb)? as u16;
        let lsb = self.read_register(RegPreambleLsb)? as u16;

        Ok((msb << 8) | lsb)
    }

    /// Enables are disables the radio's CRC check. Default value is `false`.
    pub fn set_crc(&mut self, value: bool) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let modem_config_2 = self.read_register(RegModemConfig2)?;
        if value {
            self.write_register(RegModemConfig2, modem_config_2 | 0x04)
        } else {
            self.write_register(RegModemConfig2, modem_config_2 & 0xfb)
        }
    }

    pub fn invert_rx_iq(&mut self, inv: bool) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let mut regiq = self.read_register(RegInvertiq)?
            & register::RFLR_INVERTIQ_TX_MASK
            & register::RFLR_INVERTIQ_RX_MASK;

        if inv {
            regiq |= register::RFLR_INVERTIQ_RX_ON | register::RFLR_INVERTIQ_TX_OFF;
        } else {
            regiq |= register::RFLR_INVERTIQ_RX_OFF | register::RFLR_INVERTIQ_TX_OFF;
        }

        self.write_register(RegInvertiq, regiq)?;
        self.write_register(
            RegInvertiq2,
            if inv {
                register::RFLR_INVERTIQ2_ON
            } else {
                register::RFLR_INVERTIQ2_OFF
            },
        )?;

        Ok(())
    }

    pub fn invert_tx_iq(&mut self, inv: bool) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let mut regiq = self.read_register(RegInvertiq)?
            & register::RFLR_INVERTIQ_TX_MASK
            & register::RFLR_INVERTIQ_RX_MASK;

        if inv {
            regiq |= register::RFLR_INVERTIQ_RX_OFF | register::RFLR_INVERTIQ_TX_ON;
        } else {
            regiq |= register::RFLR_INVERTIQ_RX_OFF | register::RFLR_INVERTIQ_TX_OFF;
        }

        self.write_register(RegInvertiq, regiq)?;
        self.write_register(
            RegInvertiq2,
            if inv {
                register::RFLR_INVERTIQ2_ON
            } else {
                register::RFLR_INVERTIQ2_OFF
            },
        )?;

        Ok(())
    }

    /// Calculates the symbol time in µs from current radio setup
    pub fn get_symbol_time_us(&mut self) -> Result<u32, Error<E, CS::Error, RESET::Error>> {
        let bw = self.get_signal_bandwidth()?;
        let sf = self.get_spreading_factor()?;

        Ok(calc_symbol_time_us(sf, bw))
    }

    /// Reads and calculates data rate in bytes/s from current radio setup
    pub fn get_data_rate_bps(&mut self) -> Result<u32, Error<E, CS::Error, RESET::Error>> {
        let bw = self.get_signal_bandwidth()?;
        let sf = self.get_spreading_factor()?;
        let cr = self.get_coding_rate_4()?;

        Ok(calc_data_rate(sf, bw, cr))
    }

    /// Returns the spreading factor of the radio.
    pub fn get_spreading_factor(&mut self) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        Ok(self.read_register(RegModemConfig2)? >> 4)
    }

    /// Returns the signal bandwidth of the radio.
    pub fn get_signal_bandwidth(&mut self) -> Result<BwSetting, Error<E, CS::Error, RESET::Error>> {
        let regval = self.read_register(RegModemConfig1)? >> 4;
        let bw = BwSetting::try_from(regval).map_err(|_| Error::IllegalBwSetting)?;

        Ok(bw)
    }

    /// Returns the RSSI of the last received packet.
    pub fn get_packet_rssi(&mut self) -> Result<i32, Error<E, CS::Error, RESET::Error>> {
        Ok(i32::from(self.read_register(RegPktRssiValue)?) - 157) // -139 for SX1272
    }

    /// Returns the signal to noise radio of the the last received packet.
    pub fn get_packet_snr(&mut self) -> Result<f32, Error<E, CS::Error, RESET::Error>> {
        Ok(f32::from(self.read_register(RegPktSnrValue)?) / 4.0)
    }

    /// Returns the frequency error of the last received packet in Hz.
    pub fn get_packet_frequency_error(&mut self) -> Result<i64, Error<E, CS::Error, RESET::Error>> {
        let mut freq_error: i32 = 0;
        freq_error = i32::from(self.read_register(RegFreqErrorMsb)? & 0x7);
        freq_error <<= 8i64;
        freq_error += i32::from(self.read_register(RegFreqErrorMid)?);
        freq_error <<= 8i64;
        freq_error += i32::from(self.read_register(RegFreqErrorLsb)?);

        let f_xtal = 32_000_000; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
        let f_error = ((f64::from(freq_error) * (1i64 << 24) as f64) / f64::from(f_xtal))
            * (self.get_signal_bandwidth()?.as_hz() as f64 / 500_000.0f64); // p. 37
        Ok(f_error as i64)
    }

    fn set_ldo_flag(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let sw = self.get_signal_bandwidth()?.as_hz() as i64;
        // Section 4.1.1.5
        let symbol_duration = 1000 / (sw / ((1 as i64) << self.get_spreading_factor()?));

        // Section 4.1.1.6
        let ldo_on = symbol_duration > 16;

        let mut config_3 = self.read_register(RegModemConfig3)?;
        config_3.set_bit(3, ldo_on);
        self.write_register(RegModemConfig3, config_3)
    }

    pub fn read_irq_flags(&mut self) -> Result<IrqFlags, Error<E, CS::Error, RESET::Error>> {
        let irq_flags = self.read_register(RegIrqFlags)?;
        Ok(irq_flags.into())
    }

    fn read_register<REG: Into<u8>>(
        &mut self,
        reg: REG,
    ) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        self.cs.set_low().map_err(CS)?;

        let mut buffer = [reg.into() & 0x7f, 0];
        let transfer = self.spi.transfer(&mut buffer).map_err(SPI)?;
        self.cs.set_high().map_err(CS)?;

        Ok(transfer[1])
    }

    fn write_register<REG: Into<u8>>(
        &mut self,
        reg: REG,
        byte: u8,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        self.cs.set_low().map_err(CS)?;

        let buffer = [reg.into() | 0x80, byte];
        self.spi.write(&buffer).map_err(SPI)?;
        self.cs.set_high().map_err(CS)?;

        Ok(())
    }

    pub fn put_in_fsk_mode(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let mut op_mode: u8 = 0x0;

        op_mode
            .set_bit(7, false) // FSK mode
            .set_bits(5..6, 0x00) // FSK modulation
            .set_bit(3, false) //Low freq registers
            .set_bits(0..2, 0b011); // Mode

        self.write_register(RegOpMode, op_mode)
    }

    pub fn set_fsk_pa_ramp(
        &mut self,
        modulation_shaping: FskDataModulationShaping,
        ramp: FskRampUpRamDown,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let mut pa_ramp: u8 = 0x0;
        pa_ramp
            .set_bits(5..6, modulation_shaping as u8)
            .set_bits(0..3, ramp as u8);

        self.write_register(RegPaRamp, pa_ramp)
    }
}
