//! UBX CFG-CFG "save configuration" and CFG-RST "cold reset" frames, as gpsctl's ubxSaveConfig() / ubxReset() build them.

use crate::cfg_nmea::frame;

/// Save ioPort, msgConf, navConf, rxmConf and antConf (mask 0x041B) to battery-backed RAM, flash, EEPROM and SPI flash (device mask 0x17: the receiver ignores devices it doesn't have).
pub fn save_frame() -> Vec<u8> {
    save_frame_mask(SAVE_MASK)
}

/// The mask gpsctl saves: ioPort 0x01, msgConf 0x02, navConf 0x08, rxmConf 0x10, antConf 0x400.
pub const SAVE_MASK: u32 = 0x041B;

/// deviceMask for a save: BBR | flash | EEPROM | SPI flash.
pub const DEVICE_MASK: u8 = 0x17;

/// The same CFG-CFG save with another section mask (an experiment: which section holds which setting?).
pub fn save_frame_mask(mask: u32) -> Vec<u8> {
    let mut b = [0u8; 13];
    b[4..8].copy_from_slice(&mask.to_le_bytes());
    b[12] = DEVICE_MASK;
    frame(0x06, 0x09, &b)
}

/// Cold start (navBbrMask 0xFFFF), immediate restart.  The receiver never acknowledges this.
pub fn reset_frame() -> Vec<u8> {
    frame(0x06, 0x04, &[0xFF, 0xFF, 0, 0])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn save_masks() {
        assert_eq!(save_frame(), save_frame_mask(0x041B));
        let f = save_frame_mask(0x0008);
        assert_eq!(&f[6..f.len() - 2], &[0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0x17]);
    }
}

