//! Golden vectors copied from the pinned OpenBSW CRC test fixture.

use bsw_util::crc::{
    CRC16_CCITT, CRC32_ARE2EP4, CRC32_ETHERNET, CRC8_CCITT, CRC8_H2F, CRC8_MAXIM, CRC8_ROHC,
    CRC8_SAEJ1850_OPENBSW,
};

const ONE: &[u8] = &[0x31];
const ZERO: &[u8] = &[0x00];
const CHECK: &[u8] = b"123456789";
const NINE_ZEROES: &[u8] = &[0; 9];

#[test]
fn crc8_vectors_match_openbsw() {
    let vectors = [ONE, ZERO, CHECK, NINE_ZEROES];
    for (calculator, expected) in [
        (&CRC8_CCITT, [0x97, 0x00, 0xF4, 0x00]),
        (&CRC8_SAEJ1850_OPENBSW, [0x57, 0x00, 0x37, 0x00]),
        (&CRC8_MAXIM, [0xE0, 0x00, 0xA1, 0x00]),
        (&CRC8_H2F, [0x4F, 0xBD, 0xDF, 0xE1]),
        (&CRC8_ROHC, [0x7A, 0xCF, 0xD0, 0xF0]),
    ] {
        for (input, expected) in vectors.iter().zip(expected) {
            assert_eq!(calculator.checksum(input), expected);
        }
    }
}

#[test]
fn crc16_vectors_match_openbsw() {
    for (input, expected) in [
        (ONE, 0xC782),
        (ZERO, 0xE1F0),
        (CHECK, 0x29B1),
        (NINE_ZEROES, 0x1872),
    ] {
        assert_eq!(CRC16_CCITT.checksum(input), expected);
    }
}

#[test]
fn crc32_vectors_match_openbsw() {
    for (calculator, expected) in [
        (
            &CRC32_ETHERNET,
            [0x83DC_EFB7, 0xD202_EF8D, 0xCBF4_3926, 0xE609_14AE],
        ),
        (
            &CRC32_ARE2EP4,
            [0x2DE7_AF5E, 0x6016_DC99, 0x1697_D06A, 0x4D43_C7AA],
        ),
    ] {
        for (input, expected) in [ONE, ZERO, CHECK, NINE_ZEROES].iter().zip(expected) {
            assert_eq!(calculator.checksum(input), expected);
        }
    }
}
