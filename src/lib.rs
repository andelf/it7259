//! IT7259 is a single-chip capacitive touch controller for touch screen applications.
//! It supports up to 10 touch points and gesture recognition with a built-in 32-bit CPU.
//! It also supports I2C interface and firmware update.
//! This crate provides a platform agnostic driver for the IT7259 touch controller.
//! It was built using [`embedded-hal`] traits.

#![no_std]

use embedded_hal_1::{delay::DelayUs, digital::OutputPin, i2c::I2c};

pub const DEFAULT_ADDR: u8 = 0x46;

// 4.3. Buffer Type and Format
/// Command Buffer
const BUFFER_TYPE_COMMAND: u8 = 0b001_00000; // 0x20
/// Query Buffer, read only
const BUFFER_TYPE_QUERY: u8 = 0b100_00000; // 0x80
/// Command Response Buffer, read only. 1 byte
const BUFFER_TYPE_RESPONSE: u8 = 0b101_00000; // 0xa0
/// Point Information Buffer, read only
const BUFFER_TYPE_POINT_INFO: u8 = 0b111_00000; // 0xe0

pub mod commands {
    pub const DEVICE_NAME: u8 = 0x00;

    /// Get Cap Sensor Information
    pub const GET_SENSOR_INFO: u8 = 0x01;
    pub const SET_SENSOR_INFO: u8 = 0x02;

    pub const SET_POWER_MODE: u8 = 0x04;

    pub mod sensor_info {
        pub const FIRMWARE_INFOMATION: u8 = 0x00;
        pub const RESOLUTIONS_2D: u8 = 0x02;
        pub const FLASH_SIZE: u8 = 0x03;
        pub const INTERRUPT_NOTIFICATION_STATUS: u8 = 0x04; // rw
        pub const GESTURE_INFO: u8 = 0x05;
        pub const CONFIGURATION_VERSION: u8 = 0x06;
    }
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PointReport {
    /// Finger or Pen
    pub finger: bool,
    /// Palm detected
    pub palm: bool,
    /// x, y, pressure(0, 1, 2, 4, 8, f)
    pub point1: Option<(u16, u16, u8)>,
    pub point2: Option<(u16, u16, u8)>,
    pub point3: Option<(u16, u16, u8)>,
}

impl PointReport {
    fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf[0] >> 4 != 0 {
            return None;
        }

        let finger = buf[0] & 0b1000 != 0;
        let palm = buf[1] & 0b1 != 0;

        let point1 = if buf[0] & 0b001 != 0 {
            let x = u16::from(buf[2]) | ((u16::from(buf[3]) & 0b1111) << 8);
            let y = u16::from(buf[4]) | ((u16::from(buf[3]) & 0b1111_0000) << 4);
            let pressure = buf[5] & 0b1111;
            Some((x, y, pressure))
        } else {
            None
        };
        let point2 = if buf[0] & 0b010 != 0 {
            let x = u16::from(buf[6]) | ((u16::from(buf[7]) & 0b1111) << 8);
            let y = u16::from(buf[8]) | ((u16::from(buf[7]) & 0b1111_0000) << 4);
            let pressure = buf[9] & 0b1111;
            Some((x, y, pressure))
        } else {
            None
        };
        let point3 = if buf[0] & 0b100 != 0 {
            let x = u16::from(buf[10]) | ((u16::from(buf[11]) & 0b1111) << 8);
            let y = u16::from(buf[12]) | ((u16::from(buf[11]) & 0b1111_0000) << 4);
            let pressure = buf[13] & 0b1111;
            Some((x, y, pressure))
        } else {
            None
        };
        Some(Self {
            finger,
            palm,
            point1,
            point2,
            point3,
        })
    }
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Gesture {
    /// Tap, GID: 0x20
    Tap {
        x: u16,
        y: u16,
    },
    /// Press, GID: 0x21
    Press {
        x: u16,
        y: u16,
    },
    /// Flick, GID: 0x22
    Flick {
        start_x: u16,
        start_y: u16,
        end_x: u16,
        end_y: u16,
        direction: u8,
    },
    /// Double-Tap, GID: 0x23
    DoubleTap {
        x: u16,
        y: u16,
    },
    /// Tap-and-Slide, GID: 0x24
    TapAndSlide {
        x: u16,
        y: u16,
    },
    /// Drag, GID: 0x25
    Drag {
        x: u16,
        y: u16,
    },
    /// Direction, GID: 0x26
    /// - 000b Up
    /// - 001b Upper right
    /// - 010b Right
    /// - 011b Lower right
    /// - 100b Down
    /// - 101b Lower left
    /// - 110b Left
    /// - 111b Upper left
    Direction {
        direction: u8,
    },
    /// Turn, GID: 0x27
    Turn {
        start_direction: u8,
        end_direction: u8,
    },
    /// Clockwise, GID: 0x28
    Clockwise {
        cw: bool,
        count: u8,
    },
    /// Dir_4Way, GID: 0x29
    Dir4Way {
        direction: u8,
    },

    // 2-finger gestures
    /// 2-finger Tap, GID: 0x40
    Tap2Finger {
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
    },
    /// 2-finger Double Tap, GID: 0x41
    DoubleTap2Finger {
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
    },

    /// 2D Gesture (Rotate + Scale + Translate), GID: 0x42
    Gesture2D {
        x_trans: u16,
        y_trans: u16,
        scale: u16,
        rorate: i16,
    },
    Unknown(u8),
}

impl Gesture {
    fn from_bytes(raw: &[u8]) -> Self {
        let gid = raw[0];
        match gid {
            0x20 => {
                let x = u16::from_le_bytes(raw[1..3].try_into().unwrap());
                let y = u16::from_le_bytes(raw[3..5].try_into().unwrap());
                Self::Tap { x, y }
            }
            0x21 => {
                let x = u16::from_le_bytes(raw[1..3].try_into().unwrap());
                let y = u16::from_le_bytes(raw[3..5].try_into().unwrap());
                Self::Press { x, y }
            }
            0x22 => {
                let start_x = u16::from_le_bytes(raw[1..3].try_into().unwrap());
                let start_y = u16::from_le_bytes(raw[3..5].try_into().unwrap());
                let end_x = u16::from_le_bytes(raw[5..7].try_into().unwrap());
                let end_y = u16::from_le_bytes(raw[7..9].try_into().unwrap());
                let direction = raw[9];
                Self::Flick {
                    start_x,
                    start_y,
                    end_x,
                    end_y,
                    direction,
                }
            }
            0x23 => {
                let x = u16::from_le_bytes(raw[1..3].try_into().unwrap());
                let y = u16::from_le_bytes(raw[3..5].try_into().unwrap());
                Self::DoubleTap { x, y }
            }
            0x24 => {
                let x = u16::from_le_bytes(raw[1..3].try_into().unwrap());
                let y = u16::from_le_bytes(raw[3..5].try_into().unwrap());
                Self::TapAndSlide { x, y }
            }
            0x25 => {
                let x = u16::from_le_bytes(raw[1..3].try_into().unwrap());
                let y = u16::from_le_bytes(raw[3..5].try_into().unwrap());
                Self::Drag { x, y }
            }
            0x26 => {
                let direction = raw[1] & 0b111;
                Self::Direction { direction }
            }
            0x27 => {
                let start_direction = raw[1] & 0b111;
                let end_direction = (raw[1] >> 4) & 0b111;
                Self::Turn {
                    start_direction,
                    end_direction,
                }
            }
            0x28 => {
                let cw = raw[1] >> 7 != 0;
                let count = (raw[1] >> 4) & 0b111;
                Self::Clockwise { cw, count }
            }
            0x29 => {
                let direction = raw[1] & 0b111;
                Self::Dir4Way { direction }
            }
            0x40 => {
                let x0 = u16::from_le_bytes(raw[1..3].try_into().unwrap());
                let y0 = u16::from_le_bytes(raw[3..5].try_into().unwrap());
                let x1 = u16::from_le_bytes(raw[5..7].try_into().unwrap());
                let y1 = u16::from_le_bytes(raw[7..9].try_into().unwrap());
                Self::Tap2Finger { x0, y0, x1, y1 }
            }
            _ => Self::Unknown(gid),
        }
    }
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchReport {
    pub tid: u8,
    pub touch_type: u8,
    pub info: [u8; 11],
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    Point(PointReport),
    Gesture(Gesture),
    Touch(TouchReport),
    Wakeup,
}

impl Event {
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        match buf[0] >> 4 {
            0b0000 => PointReport::from_bytes(buf).map(Self::Point),
            0b1000 => {
                // let gid = buf[0] & 0b1111;
                Some(Self::Gesture(Gesture::from_bytes(&buf[1..])))
            }
            0b0100 => Some(Self::Touch(TouchReport {
                tid: buf[0] & 0b1111,
                touch_type: buf[1],
                info: [
                    buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12],
                ],
            })),
            0b1100 => Some(Self::Wakeup),
            _ => None,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum InterruptTrigger {
    Low = 0x00,
    High = 0x01,
    FallingEdge = 0x10,
    RisingEdge = 0x11,
}

impl Default for InterruptTrigger {
    fn default() -> Self {
        Self::Low
    }
}

impl From<u8> for InterruptTrigger {
    fn from(v: u8) -> Self {
        match v {
            0x00 => Self::Low,
            0x01 => Self::High,
            0x10 => Self::FallingEdge,
            0x11 => Self::RisingEdge,
            _ => unreachable!(),
        }
    }
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum IdleGesture {
    Point = 0x00,
    Tap = 0x20,
    DoubleTap = 0x23,
    Palm = 0x80,
}

pub struct IT7259<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C> IT7259<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }

    pub fn hard_reset<P: OutputPin, D: DelayUs>(&self, reset_n: &mut P, delay: &mut D) -> Result<(), P::Error> {
        reset_n.set_low()?;
        delay.delay_us(10);
        reset_n.set_high()?;
        delay.delay_ms(100);

        Ok(())
    }

    pub fn init(&mut self) -> Result<(), I2C::Error> {
        // 0x46 = 0b1000_110

        let mut buf = [0u8; 0xf];
        self.transfer_command(commands::DEVICE_NAME, &mut buf)?;
        let raw = &buf[1..buf[0] as usize];

        if &raw[0..3] != b"ITE" {
            todo!();
        }
        if &raw[3..6] != b"7259" && &raw[3..6] != b"7257" {
            todo!();
        }
        Ok(())
    }

    pub fn poll_event(&mut self) -> Result<Option<Event>, I2C::Error> {
        let query = self.get_query_buffer()?;
        let point_info = query >> 6;
        if point_info & 0b10 != 0 {
            let mut buf = [0u8; 14];
            self.i2c
                .write_read(self.addr, &[BUFFER_TYPE_POINT_INFO], &mut buf[..])?;

            if buf[0] != 0 {
                return Ok(Event::from_bytes(&buf[..]));
            }
        }
        Ok(None)
    }

    /// (x, y, scale)
    pub fn get_resolution(&mut self) -> Result<(u16, u16, u8), I2C::Error> {
        let mut buf = [0u8; 0x0e];
        self.transfer_subcommand(
            commands::GET_SENSOR_INFO,
            commands::sensor_info::RESOLUTIONS_2D,
            &mut buf,
        )?;

        let x = u16::from_le_bytes([buf[2], buf[3]]);
        let y = u16::from_le_bytes([buf[4], buf[5]]);
        let scale = buf[6];

        Ok((x, y, scale))
    }

    /// Each bit presents a gesture ID.
    /// (1 finger gesture, 2 fingers gesture, 3 fingers gesture)
    /// 0: Gesture ID is not supported.
    /// 1: Gesture ID is supported.
    pub fn get_gesture_support(&mut self) -> Result<(u32, u32, u32), I2C::Error> {
        let mut buf = [0u8; 0x0e];
        self.transfer_subcommand(commands::GET_SENSOR_INFO, commands::sensor_info::GESTURE_INFO, &mut buf)?;

        let gesture_1finger = u32::from_le_bytes([buf[2], buf[3], buf[4], buf[5]]);
        let gesture_2finger = u32::from_le_bytes([buf[6], buf[7], buf[8], buf[9]]);
        let gesture_3finger = u32::from_le_bytes([buf[10], buf[11], buf[12], buf[13]]);

        Ok((gesture_1finger, gesture_2finger, gesture_3finger))
    }

    pub fn get_interrupt(&mut self) -> Result<Option<InterruptTrigger>, I2C::Error> {
        let mut buf = [0u8; 2];
        self.transfer_raw_command(
            &[
                BUFFER_TYPE_COMMAND,
                commands::GET_SENSOR_INFO,
                commands::sensor_info::INTERRUPT_NOTIFICATION_STATUS,
            ],
            &mut buf,
        )?;
        #[cfg(feature = "defmt")]
        defmt::info!("get_interrupt: {:?}", buf);
        if buf == [0, 0] {
            Ok(None)
        } else {
            Ok(Some(buf[1].into()))
        }
    }

    pub fn set_interrupt(&mut self, trigger: Option<InterruptTrigger>) -> Result<bool, I2C::Error> {
        let mut buf = [0u8; 2];
        if let Some(trigger) = trigger {
            self.transfer_raw_command(
                &[
                    BUFFER_TYPE_COMMAND,
                    commands::SET_SENSOR_INFO,
                    commands::sensor_info::INTERRUPT_NOTIFICATION_STATUS,
                    0x01,
                    trigger as u8,
                ],
                &mut buf,
            )?;
        } else {
            self.transfer_raw_command(
                &[
                    BUFFER_TYPE_COMMAND,
                    commands::SET_SENSOR_INFO,
                    commands::sensor_info::INTERRUPT_NOTIFICATION_STATUS,
                    0x00,
                    0x00,
                ],
                &mut buf,
            )?;
        }
        Ok(buf == [0, 0])
    }

    #[inline]
    fn get_query_buffer(&mut self) -> Result<u8, I2C::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[BUFFER_TYPE_QUERY], &mut buf[..1])?;
        Ok(buf[0])
    }

    // From active mode to idle mode,
    pub fn enter_idle(&mut self) -> Result<(), I2C::Error> {
        // no response
        self.i2c.write(
            self.addr,
            &[
                BUFFER_TYPE_COMMAND,
                commands::SET_POWER_MODE,
                0x00,
                0x01, // Idle mode
            ],
        )?;
        Ok(())
    }

    pub fn set_idle_gesture(&mut self, gesture: Option<IdleGesture>) -> Result<bool, I2C::Error> {
        // 5.3.17.5. Sub Command 0x87: Idle Gesture Setting
        const COMMAND: u8 = 0x15;
        const SUBCOMMAND_IDLE_GESTURE_SETTING: u8 = 0x87;

        let mut buf = [0u8; 2];
        if let Some(gesture) = gesture {
            self.transfer_raw_command(
                &[
                    BUFFER_TYPE_COMMAND,
                    COMMAND,
                    SUBCOMMAND_IDLE_GESTURE_SETTING,
                    0x00,
                    0x00,
                    gesture as u8,
                    0x80, // enable
                ],
                &mut buf,
            )?;
        } else {
            self.transfer_raw_command(
                &[
                    BUFFER_TYPE_COMMAND,
                    COMMAND,
                    SUBCOMMAND_IDLE_GESTURE_SETTING,
                    0x00,
                    0x00,
                    0x00,
                    0x00, // disable
                ],
                &mut buf,
            )?;
        }
        Ok(buf == [0, 0])
    }

    fn transfer_command(&mut self, command: u8, buf: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[BUFFER_TYPE_COMMAND, command])?;
        loop {
            self.i2c.write_read(self.addr, &[BUFFER_TYPE_QUERY], &mut buf[..1])?;
            if buf[0] & 0b11 == 0 {
                break;
            } else if buf[0] & 0b11 == 0b01 {
                // busy
            } else {
                #[cfg(feature = "defmt")]
                defmt::error!("error {}", buf[0]);
                todo!();
            }
        }
        self.i2c.write_read(self.addr, &[BUFFER_TYPE_RESPONSE], buf)?;
        Ok(())
    }

    fn transfer_subcommand(&mut self, command: u8, subcommand: u8, buf: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[BUFFER_TYPE_COMMAND, command, subcommand])?;
        loop {
            self.i2c.write_read(self.addr, &[BUFFER_TYPE_QUERY], &mut buf[..1])?;
            if buf[0] & 0b11 == 0 {
                break;
            } else if buf[0] & 0b11 == 0b01 {
                // busy
            } else {
                #[cfg(feature = "defmt")]
                defmt::error!("error {}", buf[0]);
                todo!();
            }
        }
        self.i2c.write_read(self.addr, &[BUFFER_TYPE_RESPONSE], buf)?;
        Ok(())
    }

    fn transfer_raw_command(&mut self, bytes: &[u8], buf: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, bytes)?;

        loop {
            self.i2c.write_read(self.addr, &[BUFFER_TYPE_QUERY], &mut buf[..1])?;
            let query_data = buf[0];
            if query_data & 0b11 == 0 {
                break;
            } else if query_data & 0b11 == 0b01 {
                // busy
            } else {
                #[cfg(feature = "defmt")]
                defmt::error!("error {}", query_data);
                todo!();
            }
        }

        self.i2c.write_read(self.addr, &[BUFFER_TYPE_RESPONSE], buf)?;
        Ok(())
    }
}
