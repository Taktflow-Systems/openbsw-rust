//! ISO-TP addressing types.
//!
//! Separates the transport-level logical addresses (used by upper layers) from
//! the data-link-level CAN IDs (used by the frame codec and CAN driver).

use crate::codec::CodecConfig;

/// ISO 15765-2 addressing format selected for one connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AddressingFormat {
    /// N_AI is encoded entirely by the CAN identifier.
    Normal,
    /// First data byte contains the target/source address.
    Extended,
    /// First data byte contains the address extension (11- or 29-bit CAN).
    Mixed,
}

/// Data-byte addressing configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Addressing {
    /// Addressing format.
    pub format: AddressingFormat,
    /// Required first data byte for extended/mixed addressing.
    pub extension: u8,
}

impl Addressing {
    /// Normal addressing has no data-byte prefix.
    pub const fn normal() -> Self {
        Self {
            format: AddressingFormat::Normal,
            extension: 0,
        }
    }

    /// Extended addressing with one target/source address byte.
    pub const fn extended(extension: u8) -> Self {
        Self {
            format: AddressingFormat::Extended,
            extension,
        }
    }

    /// Mixed addressing with one address-extension byte.
    pub const fn mixed(extension: u8) -> Self {
        Self {
            format: AddressingFormat::Mixed,
            extension,
        }
    }

    /// Codec offset for this format.
    pub const fn pci_offset(self) -> usize {
        match self.format {
            AddressingFormat::Normal => 0,
            AddressingFormat::Extended | AddressingFormat::Mixed => 1,
        }
    }

    /// Build a codec configuration and selected filler byte.
    pub const fn codec(self, filler_byte: u8) -> CodecConfig {
        CodecConfig {
            pci_offset: self.pci_offset(),
            filler_byte,
        }
    }

    /// Write the format prefix into an outgoing frame.
    pub fn write_prefix(self, frame: &mut [u8]) -> bool {
        if self.pci_offset() == 0 {
            true
        } else if let Some(first) = frame.first_mut() {
            *first = self.extension;
            true
        } else {
            false
        }
    }

    /// Check the format prefix of an incoming frame.
    pub fn accepts(self, frame: &[u8]) -> bool {
        self.pci_offset() == 0 || frame.first().is_some_and(|byte| *byte == self.extension)
    }
}

/// Transport-level address pair identifying a communication session by logical addresses.
///
/// These map to ISO 15765-2 source and target addresses (`N_SA` / `N_TA`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TransportAddressPair {
    /// Logical source address (`N_SA`).
    pub source: u16,
    /// Logical target address (`N_TA`).
    pub target: u16,
}

impl TransportAddressPair {
    /// Construct a new address pair.
    pub const fn new(source: u16, target: u16) -> Self {
        Self { source, target }
    }
}

/// Data-link-level address pair — the actual CAN IDs used for reception and transmission.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DataLinkAddressPair {
    /// CAN identifier used for *receiving* frames (RX filter).
    pub reception_id: u32,
    /// CAN identifier used for *transmitting* frames.
    pub transmission_id: u32,
}

impl DataLinkAddressPair {
    /// Construct a new CAN ID pair.
    pub const fn new(reception_id: u32, transmission_id: u32) -> Self {
        Self {
            reception_id,
            transmission_id,
        }
    }
}

/// Combined addressing info tying logical addresses to physical CAN IDs.
///
/// Used to look up or create ISO-TP sessions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConnectionInfo {
    /// Transport-layer logical address pair.
    pub transport: TransportAddressPair,
    /// Data-link-layer CAN ID pair.
    pub data_link: DataLinkAddressPair,
}

impl ConnectionInfo {
    /// Construct a new connection info value.
    pub const fn new(transport: TransportAddressPair, data_link: DataLinkAddressPair) -> Self {
        Self {
            transport,
            data_link,
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transport_address_pair_construction() {
        let pair = TransportAddressPair::new(0x0001, 0x0002);
        assert_eq!(pair.source, 0x0001);
        assert_eq!(pair.target, 0x0002);
    }

    #[test]
    fn data_link_address_pair_construction() {
        let pair = DataLinkAddressPair::new(0x7DF, 0x7E8);
        assert_eq!(pair.reception_id, 0x7DF);
        assert_eq!(pair.transmission_id, 0x7E8);
    }

    #[test]
    fn connection_info_construction() {
        let tp = TransportAddressPair::new(1, 2);
        let dl = DataLinkAddressPair::new(0x7DF, 0x7E8);
        let ci = ConnectionInfo::new(tp, dl);
        assert_eq!(ci.transport.source, 1);
        assert_eq!(ci.data_link.reception_id, 0x7DF);
    }

    #[test]
    fn address_pair_equality() {
        let a = TransportAddressPair::new(1, 2);
        let b = TransportAddressPair::new(1, 2);
        let c = TransportAddressPair::new(1, 3);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn data_link_hash_derives() {
        // Verify that Hash + Eq are derived (both instances with same fields are equal).
        let a = DataLinkAddressPair::new(0x100, 0x200);
        let b = DataLinkAddressPair::new(0x100, 0x200);
        let c = DataLinkAddressPair::new(0x100, 0x201);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn normal_extended_and_mixed_offsets_and_filters() {
        let normal = Addressing::normal();
        assert_eq!(normal.pci_offset(), 0);
        assert!(normal.accepts(&[]));

        let extended = Addressing::extended(0xf1);
        assert_eq!(extended.pci_offset(), 1);
        assert!(extended.accepts(&[0xf1, 0x03]));
        assert!(!extended.accepts(&[0xf2, 0x03]));

        let mixed = Addressing::mixed(0x99);
        let mut frame = [0; 8];
        assert!(mixed.write_prefix(&mut frame));
        assert_eq!(frame[0], 0x99);
        assert!(mixed.accepts(&frame));
    }
}
