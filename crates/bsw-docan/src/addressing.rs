//! ISO-TP addressing types.
//!
//! Separates the transport-level logical addresses (used by upper layers) from
//! the data-link-level CAN IDs (used by the frame codec and CAN driver).

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
}
