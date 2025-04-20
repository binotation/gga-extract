//! Detect if a NMEA sentence is GGA and parse specific fields.
//! These functions are for a specific usecase and make lots of assumptions in the name of performance.
#![no_std]

/// Determines if the sentence is a GGA sentence.
#[inline(always)]
pub fn is_gga(buffer: &[u8; 1024], sentence_begin: usize) -> bool {
    unsafe {
        *buffer.get_unchecked((sentence_begin + 3) & 1023) == b'G'
            && *buffer.get_unchecked((sentence_begin + 4) & 1023) == b'G'
            && *buffer.get_unchecked((sentence_begin + 5) & 1023) == b'A'
    }
}

#[inline]
pub fn calculate_sentence_length(ndtr: u16, sentence_begin: usize) -> u16 {
    if (sentence_begin as u16) + ndtr < 1024 {
        1024 - ndtr - (sentence_begin as u16)
    } else {
        (1024 - (sentence_begin as u16)) + (1024 - ndtr)
    }
}

const POW10_10_DIGITS: [u32; 10] = [
    1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1,
];

/// Extracts position data from a GGA (Global Positioning System Fix Data) sentence in a circular buffer.
///
/// ### Arguments
/// * `buffer` - A 1024-byte circular buffer containing NMEA 0183 data.
/// * `sentence_begin` - Starting index of the GGA sentence in the buffer.
/// * `position_block` - Output buffer where parsed position data will be stored (10 bytes).
///
/// ### Returns
/// If the sentence contains a GNSS fix.
pub fn extract_gga(
    buffer: &[u8; 1024],
    sentence_begin: usize,
    position_block: &mut [u8; 10],
) -> bool {
    // Start in time field
    let mut i = (sentence_begin + 7) & 1023;

    unsafe {
        if *buffer.get_unchecked(i) == b',' {
            // No time field, assume no fix
            return false;
        }

        // Skip time field: go into latitude field
        const TIME_FIELD_SKIP: usize = 11;
        i = (i + TIME_FIELD_SKIP) & 1023;

        if *buffer.get_unchecked(i) == b',' {
            // No latitude field, no fix
            return false;
        }

        {
            // Parse latitude
            let mut lat: u32 = 0;
            lat += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[0];
            i = (i + 1) & 1023;
            lat += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[1];
            i = (i + 1) & 1023;
            lat += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[2];
            i = (i + 1) & 1023;
            lat += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[3];
            i = (i + 2) & 1023; // Skip decimal point
            lat += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[4];
            i = (i + 1) & 1023;
            lat += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[5];
            i = (i + 1) & 1023;
            lat += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[6];
            i = (i + 1) & 1023;
            lat += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[7];
            i = (i + 1) & 1023;
            lat += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[8];

            position_block[0] = (lat >> 24) as u8;
            position_block[1] = (lat >> 16) as u8;
            position_block[2] = (lat >> 8) as u8;
            position_block[3] = lat as u8;
            i = (i + 2) & 1023;
        }

        // Latitude hemisphere
        position_block[8] = ((*buffer.get_unchecked(i) == b'N') as u8) << 1;
        i = (i + 2) & 1023;

        {
            // Parse longitude
            let mut lon: u32 = 0;
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[0];
            i = (i + 1) & 1023;
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[1];
            i = (i + 1) & 1023;
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[2];
            i = (i + 1) & 1023;
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[3];
            i = (i + 1) & 1023;
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[4];
            i = (i + 2) & 1023; // Skip decimal point
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[5];
            i = (i + 1) & 1023;
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[6];
            i = (i + 1) & 1023;
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[7];
            i = (i + 1) & 1023;
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[8];
            i = (i + 1) & 1023;
            lon += ((*buffer.get_unchecked(i) - b'0') as u32) * POW10_10_DIGITS[9];

            position_block[4] = (lon >> 24) as u8;
            position_block[5] = (lon >> 16) as u8;
            position_block[6] = (lon >> 8) as u8;
            position_block[7] = lon as u8;
            i = (i + 2) & 1023;
        }

        // Longitude hemisphere
        position_block[8] |= (*buffer.get_unchecked(i) == b'E') as u8;
        i = (i + 7) & 1023;

        {
            let mut hdop = 0;
            if buffer[(i + 1) & 1023] == b'.' {
                // Integer part is single digit
                hdop += (*buffer.get_unchecked(i) - b'0') * 10;
                i = (i + 2) & 1023; // Skip decimal point
                hdop += *buffer.get_unchecked(i) - b'0';
            } else {
                // Integer part is double digit
                hdop = hdop.saturating_add((*buffer.get_unchecked(i) - b'0').saturating_mul(100));
                i = (i + 1) & 1023;
                hdop = hdop.saturating_add((*buffer.get_unchecked(i) - b'0') * 10);
                i = (i + 2) & 1023; // Skip decimal point
                hdop = hdop.saturating_add(*buffer.get_unchecked(i) - b'0');
            }
            position_block[9] = hdop;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::prelude::*;

    fn shift_buffer(buffer: &mut [u8; 1024], sentence: &[u8], dest: usize) {
        let mut i = dest;
        for &b in sentence {
            unsafe {
                *buffer.get_unchecked_mut(i) = b;
            }
            i = (i + 1) & 1023;
        }
    }

    #[test]
    fn test_is_gga() {
        let mut buffer: [u8; 1024] = [0; 1024];
        for i in 0..1024 {
            shift_buffer(&mut buffer, &GGA_WITH_TIME_WITH_FIX[0].0, i);
            assert!(!is_gga(&buffer, (i.wrapping_sub(1)) & 1023));
            assert!(is_gga(&buffer, i));
            assert!(!is_gga(&buffer, (i + 1) & 1023));
        }
    }

    #[test]
    fn test_calculate_sentence_length() {
        let mut rng = rand::rng();
        let mut ndtr: u16 = 1024;
        let mut sentence_begin = 0;

        for _ in 0..1000000 {
            let actual_sentence_length: u16 = rng.random_range(1..83);
            ndtr = ndtr.wrapping_sub(actual_sentence_length) & 1023;
            let sentence_length = calculate_sentence_length(ndtr, sentence_begin);
            assert_eq!(actual_sentence_length, sentence_length);
            sentence_begin = (sentence_begin + sentence_length as usize) & 1023;
        }
    }

    #[test]
    fn test_no_time_no_fix() {
        let mut position_block = [0; 10];
        let mut buffer: [u8; 1024] = [0; 1024];
        for i in 0..1024 {
            shift_buffer(&mut buffer, &GGA_NO_TIME_NO_FIX, i);
            let parsed = extract_gga(&buffer, i, &mut position_block);
            assert!(!parsed);
            assert_eq!(position_block, [0; 10]);
        }
    }

    #[test]
    fn test_with_time_no_fix() {
        let mut position_block = [0; 10];
        let mut buffer: [u8; 1024] = [0; 1024];
        for i in 0..1024 {
            shift_buffer(&mut buffer, &GGA_WITH_TIME_NO_FIX, i);
            let parsed = extract_gga(&buffer, i, &mut position_block);
            assert!(!parsed);
            assert_eq!(position_block, [0; 10]);
        }
    }

    #[test]
    fn test_with_time_with_fix() {
        let mut position_block = [0; 10];
        let mut buffer: [u8; 1024] = [0; 1024];
        for (sentence, expected_position_block) in GGA_WITH_TIME_WITH_FIX.iter() {
            for i in 0..1024 {
                shift_buffer(&mut buffer, sentence, i);
                let parsed = extract_gga(&buffer, i, &mut position_block);
                assert!(parsed);
                assert_eq!(position_block, *expected_position_block);
            }
        }
    }

    const GGA_NO_TIME_NO_FIX: [u8; 32] = *b"$GNGGA,,,,,,0,00,25.5,,,,,,*64\r\n";
    const GGA_WITH_TIME_NO_FIX: [u8; 42] = *b"$GNGGA,051154.000,,,,,0,00,25.5,,,,,,*7E\r\n";

    const GGA_WITH_TIME_WITH_FIX: [(&[u8], [u8; 10]); 5] = [
        (
            b"$GNGGA,051200.993,2734.21973,S,15303.08927,E,1,07,2.8,103.4,M,41.1,M,,*59\r\n",
            [162, 248, 225, 210, 91, 54, 169, 63, 1, 28],
        ),
        (
            b"$GNGGA,051337.000,2734.22815,S,15303.09174,E,1,15,0.9,84.6,M,41.1,M,,*6E\r\n",
            [162, 249, 2, 182, 91, 54, 170, 54, 1, 9],
        ),
        (
            b"$GPGGA,181501.000,3944.50086,N,10459.16654,W,1,03,2.10,84.6,M,41.1,M,,*6E\r\n",
            [235, 28, 78, 124, 62, 87, 107, 238, 2, 21],
        ),
        (
            b"$GPGGA,181501.000,3944.50086,N,00459.16654,E,1,03,9.50,84.6,M,41.1,M,,*6E\r\n",
            [235, 28, 78, 124, 2, 188, 161, 238, 3, 95],
        ),
        (
            b"$GNGGA,181501.000,3615.12012,S,06357.25158,W,1,03,39.9,84.6,M,41.1,M,,*6E\r\n",
            [215, 122, 90, 248, 37, 228, 101, 102, 0, 255],
        ),
    ];
}
