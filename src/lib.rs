#![cfg_attr(not(test), no_std)]

use embedded_hal::blocking::i2c::{Write, WriteRead};

/// 256 pages containing 32 bytes
pub const ADDRESS_LAST: usize = 256 * 32;

/// Typestate struct for M24C64-D
pub struct IdentificationPage;
/// Typestate struct for M24C64
pub struct NoIdentificationPage;

#[derive(Debug)]
pub enum Error<I> {
    /// I2C bus error
    I2C(I),
    /// Connection error (device not found)
    Conn,
    /// Address error (invalid or out of bounds)
    Address,
    /// Port error (invalid or out of bounds)
    Port,
}

#[repr(u8)]
enum Dest {
    Memory = 0xa,
    Identification = 0xb,
}

/// M26C64 configuration
///
/// Only configures the I2C address of the device
///
/// # Example (create config with other than default settings)
///
/// ```
/// use m24c64::Config;
///
/// let config = Config {
///     address: 0b101,
///     ..Config::default()
/// };
/// ```
#[derive(Default)]
pub struct Config {
    /// Chip enable address
    ///
    /// # Note
    ///
    /// This is only for the last three bits of the address, meaning E2, E1, E0 in the datasheet.
    /// E.g. if E2 = 1, E1 = 1, E0 = 0, use `0b110`. The rest of the address will be filled
    /// automatically, based on context
    pub address: u8,
}

/// M24C64 driver
pub struct M24C64<I2C, F> {
    // Device family. M24C64 (no Identification page) or M24C64D (with Identification page)
    _device_family: F,
    /// Device configuration
    config: Config,
    /// `embedded-hal` compatible I2C instance
    i2c: I2C,
    /// Command buffer
    cmd_buf: [u8; 34],
}

impl<I2C, S, F> M24C64<I2C, F>
where
    I2C: Write<u8, Error = S> + WriteRead<u8, Error = S>,
{
    /// Create a new instance of the M24C64 driver
    /// # Arguments
    ///
    /// * `i2c` - embedded-hal compatible I2C instance
    /// * `config` - The M24C64 `Config` device configuration struct
    ///
    /// # Example
    ///
    /// ```
    /// use m24c64::{M24C64, Config};
    ///
    /// let eeprom = M24C64::new(i2c, Config::default());
    /// ```
    pub fn new(i2c: I2C, config: Config) -> M24C64<I2C, NoIdentificationPage> {
        M24C64 {
            _device_family: NoIdentificationPage,
            config,
            i2c,
            cmd_buf: [0; 34],
        }
    }

    /// Create an instance of the M24C64-D device family type
    ///
    /// The M24C64-D offers an additional page, named the Identification Page (32 byte).
    /// The Identification Page can be used to store sensitive application parameters which can be
    /// (later) permanently locked in Read-only mode
    ///
    /// # Example
    ///
    /// ```
    /// use m24c64::{M24C64, Config};
    ///
    /// let eeprom = M24C64::new(i2c, Config::default()).with_id_page();
    /// ```
    pub fn with_id_page(self) -> M24C64<I2C, IdentificationPage> {
        M24C64 {
            _device_family: IdentificationPage,
            config: self.config,
            i2c: self.i2c,
            cmd_buf: self.cmd_buf,
        }
    }

    // Warning! Does not check for page wraps
    fn write_raw(&mut self, dest: Dest, address: usize, bytes: &[u8]) -> Result<(), Error<S>> {
        self.cmd_buf[0] = (address >> 8) as u8;
        self.cmd_buf[1] = (address & 0xff) as u8;
        self.cmd_buf[2..bytes.len() + 2].copy_from_slice(bytes);
        self.i2c
            .write(
                self.config.address | dest as u8,
                &self.cmd_buf[0..bytes.len() + 2],
            )
            .map_err(Error::I2C)
    }

    fn read_raw(&mut self, dest: Dest, address: usize, bytes: &mut [u8]) -> Result<(), Error<S>> {
        self.cmd_buf[0] = (address >> 8) as u8;
        self.cmd_buf[1] = (address & 0xff) as u8;
        self.i2c
            .write_read(self.config.address | dest as u8, &self.cmd_buf[0..2], bytes)
            .map_err(Error::I2C)
    }

    /// Write exactly 32 bytes to a page
    pub fn write_page(&mut self, page: u8, bytes: &[u8; 32]) -> Result<(), Error<S>> {
        self.write_raw(Dest::Memory, (page * 32) as usize, bytes)
    }

    /// Write bytes to an arbitrary location in memory
    ///
    /// Note: Checks whether buffer will fit on page and will **not** wrap
    pub fn write(&mut self, address: usize, bytes: &[u8]) -> Result<(), Error<S>> {
        let start_idx = address % 32;
        if start_idx + bytes.len() > 32 {
            return Err(Error::Address);
        }
        self.write_raw(Dest::Memory, address, bytes)
    }

    /// Read exactly one page into a buffer
    pub fn read_page(&mut self, page: u8, bytes: &mut [u8; 32]) -> Result<(), Error<S>> {
        self.read_raw(Dest::Memory, (page * 32) as usize, bytes)
    }

    /// Read a memory location into a buffer until it is full
    ///
    /// Note: Checks whether address is out of bounds and will **not** wrap
    pub fn read(&mut self, address: usize, bytes: &mut [u8]) -> Result<(), Error<S>> {
        if address + bytes.len() > ADDRESS_LAST {
            return Err(Error::Address);
        }
        self.read_raw(Dest::Memory, address, bytes)
    }
}

impl<I2C, S> M24C64<I2C, IdentificationPage>
where
    I2C: Write<u8, Error = S> + WriteRead<u8, Error = S>,
{
    /// Write bytes to an arbitrary location on the Identification page
    ///
    /// Note: Checks whether buffer will fit on page and will **not** wrap
    pub fn write_id(&mut self, mut address: usize, bytes: &[u8]) -> Result<(), Error<S>> {
        if address + bytes.len() > 32 {
            return Err(Error::Address);
        }
        // Unset address bit 10
        address &= !(1 << 10);
        self.write_raw(Dest::Identification, address, bytes)
    }

    /// Write exactly 32 bytes to the Identification page
    pub fn write_id_page(&mut self, bytes: &[u8; 32]) -> Result<(), Error<S>> {
        self.write_raw(Dest::Identification, 0, bytes)
    }

    /// Permanently locs the Identification page (this makes it read-only)
    pub fn lock_id_page(&mut self) -> Result<(), Error<S>> {
        // Set address bit 10 to `1`
        let address = 0x400;
        // Set data bit 2 to `1`;
        let data_byte = 0x2;
        self.write_raw(Dest::Identification, address, &[data_byte])
    }

    /// Read a location on the Identification page into a buffer until it is full
    ///
    /// Note: Checks whether address is out of bounds and will **not** wrap
    pub fn read_id(&mut self, address: usize, bytes: &mut [u8]) -> Result<(), Error<S>> {
        if address + bytes.len() > 32 {
            return Err(Error::Address);
        }
        self.read_raw(Dest::Identification, address, bytes)
    }

    /// Read the whole Identification page into a buffer
    pub fn read_id_page(&mut self, bytes: &mut [u8; 32]) -> Result<(), Error<S>> {
        self.read_raw(Dest::Identification, 0, bytes)
    }
}
