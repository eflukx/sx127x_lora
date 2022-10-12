#![allow(dead_code)]

use bit_field::BitField;
use core::{convert::TryFrom, fmt::Debug};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Register {
    RegFifo = 0x00,
    RegOpMode = 0x01,
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegPaRamp = 0x0a,
    RegOcp = 0x0b,
    RegLna = 0x0c,
    RegFifoAddrPtr = 0x0d,
    RegFifoTxBaseAddr = 0x0e,
    RegFifoRxBaseAddr = 0x0f,
    RegFifoRxCurrentAddr = 0x10,
    RegIrqFlagsMask = 0x11,
    RegIrqFlags = 0x12,
    RegRxNbBytes = 0x13,
    RegPktSnrValue = 0x19,
    RegPktRssiValue = 0x1a,
    RegModemConfig1 = 0x1d,
    RegModemConfig2 = 0x1e,
    RegPreambleMsb = 0x20,
    RegPreambleLsb = 0x21,
    RegPayloadLength = 0x22,
    RegModemConfig3 = 0x26,
    RegFreqErrorMsb = 0x28,
    RegFreqErrorMid = 0x29,
    RegFreqErrorLsb = 0x2a,
    RegRssiWideband = 0x2c,
    RegDetectionOptimize = 0x31,
    RegHighBWOptimize1 = 0x36,
    RegInvertiq = 0x33,
    RegDetectionThreshold = 0x37,
    RegSyncWord = 0x39,
    RegHighBWOptimize2 = 0x3a,
    RegInvertiq2 = 0x3b,
    RegDioMapping1 = 0x40,
    RegVersion = 0x42,
    RegPaDac = 0x4d,
}

impl From<Register> for u8 {
    fn from(mode: Register) -> Self {
        mode.as_byte()
    }
}

impl Register {
    pub fn as_byte(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy, PartialEq, Default)]
pub struct IrqFlags(u8);

impl From<IrqFlags> for u8 {
    fn from(value: IrqFlags) -> Self {
        value.0
    }
}

impl From<u8> for IrqFlags {
    fn from(value: u8) -> Self {
        IrqFlags(value)
    }
}

impl IrqFlags {
    pub fn as_byte(self) -> u8 {
        self.0
    }

    pub fn is_set(&self, irq: IRQ) -> bool {
        self.0 & irq.as_byte() != 0
    }

    pub fn rx_timeout(&self) -> bool {
        self.0.get_bit(7)
    }

    pub fn rx_done(&self) -> bool {
        self.0.get_bit(6)
    }

    pub fn payload_crc_error(&self) -> bool {
        self.0.get_bit(5)
    }

    pub fn valid_header(&self) -> bool {
        self.0.get_bit(4)
    }

    pub fn tx_done(&self) -> bool {
        self.0.get_bit(3)
    }

    pub fn cad_done(&self) -> bool {
        self.0.get_bit(2)
    }

    pub fn fhss_change_channel(&self) -> bool {
        self.0.get_bit(1)
    }

    pub fn cad_detected(&self) -> bool {
        self.0.get_bit(0)
    }
}

impl Debug for IrqFlags {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "IrqFlags({}) ", self.as_byte())?;

        let mut set = f.debug_set();

        if self.cad_detected() {
            set.entry(&"cad_detected");
        }
        if self.cad_done() {
            set.entry(&"cad_done");
        }
        if self.fhss_change_channel() {
            set.entry(&"fhss_change_channel");
        }
        if self.payload_crc_error() {
            set.entry(&"payload_crc_error");
        }
        if self.rx_done() {
            set.entry(&"rx_done");
        }
        if self.rx_timeout() {
            set.entry(&"rx_timeout");
        }
        if self.tx_done() {
            set.entry(&"tx_done");
        }
        if self.valid_header() {
            set.entry(&"valid_header");
        }

        set.finish()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IRQ {
    RxTimeOut = (1 << 7),
    RxDone = (1 << 6),
    PayloadCrcError = (1 << 5),
    ValidHeader = (1 << 4),
    TxDone = (1 << 3),
    CadDone = (1 << 2),
    FHSSChangeChannel = (1 << 1),
    CadDetected = (1 << 0),
}

impl From<IRQ> for u8 {
    fn from(mode: IRQ) -> Self {
        mode.as_byte()
    }
}

impl IRQ {
    pub fn as_byte(self) -> u8 {
        self as u8
    }
}

/// Modes of the radio and their corresponding register values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RadioMode {
    LongRangeMode = 0x80,
    Sleep = 0x00,
    Stdby = 0x01,
    FsTx = 0x02,
    Tx = 0x03,
    RxContinuous = 0x05,
    RxSingle = 0x06,
}

impl From<RadioMode> for u8 {
    fn from(mode: RadioMode) -> Self {
        mode.as_byte()
    }
}

impl RadioMode {
    /// Returns the address of the mode.
    pub fn as_byte(self) -> u8 {
        self as u8
    }
}

/* 0x33 RegInvertIQ */
pub(crate) const RFLR_INVERTIQ_RX_MASK: u8 = 0xBF;
pub(crate) const RFLR_INVERTIQ_RX_OFF: u8 = 0x00;
pub(crate) const RFLR_INVERTIQ_RX_ON: u8 = 0x40;
pub(crate) const RFLR_INVERTIQ_TX_MASK: u8 = 0xFE;
pub(crate) const RFLR_INVERTIQ_TX_OFF: u8 = 0x01;
pub(crate) const RFLR_INVERTIQ_TX_ON: u8 = 0x00;

/* 0x3b RegInvertIQ2 */
pub(crate) const RFLR_INVERTIQ2_ON: u8 = 0x19;
pub(crate) const RFLR_INVERTIQ2_OFF: u8 = 0x1D;

/* 0x0c REG_LNA */
pub(crate) const LNA_OFF_GAIN: u8 = 0x00;
pub(crate) const LNA_MAX_G1: u8 = 1 << 5;
pub(crate) const LNA_MAX_GAIN: u8 = 0x20;
pub(crate) const LNA_BOOST: u8 = 0x03;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PaConfig {
    PaBoost = 0x80,
    PaOutputRfoPin = 0,
}

impl From<PaConfig> for u8 {
    fn from(mode: PaConfig) -> Self {
        mode.as_byte()
    }
}

impl PaConfig {
    pub fn as_byte(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FskDataModulationShaping {
    None = 1,
    GaussianBt1d0 = 2,
    GaussianBt0d5 = 10,
    GaussianBt0d3 = 11,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FskRampUpRamDown {
    #[allow(non_camel_case_types)]
    _3d4ms = 0b000,
    _2ms = 0b0001,
    _1ms = 0b0010,
    _500us = 0b0011,
    _250us = 0b0100,
    _125us = 0b0101,
    _100us = 0b0110,
    _62us = 0b0111,
    _50us = 0b1000,
    _40us = 0b1001,
    _31us = 0b1010,
    _25us = 0b1011,
    _20us = 0b1100,
    _15us = 0b1101,
    _12us = 0b1110,
    _10us = 0b1111,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BwSetting {
    Bw7_8kHz = 0b0000,
    Bw10_4kHz = 0b0001,
    Bw15_6kHz = 0b0010,
    Bw20_8kHz = 0b0011,
    Bw31_25kHz = 0b0100,
    Bw41_7kHz = 0b0101,
    Bw62_5kHz = 0b0110,
    Bw125kHz = 0b0111,
    Bw250kHz = 0b1000,
    Bw500kHz = 0b1001,
}

impl BwSetting {
    pub fn as_hz(&self) -> u32 {
        match self {
            BwSetting::Bw7_8kHz => 7_800,
            BwSetting::Bw10_4kHz => 10_400,
            BwSetting::Bw15_6kHz => 15_600,
            BwSetting::Bw20_8kHz => 20_800,
            BwSetting::Bw31_25kHz => 31_250,
            BwSetting::Bw41_7kHz => 41_700,
            BwSetting::Bw62_5kHz => 62_500,
            BwSetting::Bw125kHz => 125_000,
            BwSetting::Bw250kHz => 250_000,
            BwSetting::Bw500kHz => 500_000,
        }
    }
}

impl TryFrom<u8> for BwSetting {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Bw7_8kHz),
            1 => Ok(Self::Bw10_4kHz),
            2 => Ok(Self::Bw15_6kHz),
            3 => Ok(Self::Bw20_8kHz),
            4 => Ok(Self::Bw31_25kHz),
            5 => Ok(Self::Bw41_7kHz),
            6 => Ok(Self::Bw62_5kHz),
            7 => Ok(Self::Bw125kHz),
            8 => Ok(Self::Bw250kHz),
            9 => Ok(Self::Bw500kHz),
            _ => Err(()),
        }
    }
}
