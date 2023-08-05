//! # Abstraction of PCA9570
//!
//! Central part of this crate is the struct [PCA9570], which either allows central I/O control or
//! or alternatively offers a breakdown into individual pins.
//!
//! The following examples demonstrates central I/O control. For getting separate pin instances,
//! see the [pins module](crate::pins).
//!
//! ## Setup
//! [PCA9570] instance is created using a I2CBus implementing the I2C traits of
//! [embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/blocking/i2c/index.html).
//!```
//! use pca9570::example::DummyI2CBus;
//! use pca9570::expander::PCA9570;
//!
//! let i2c_bus = DummyI2CBus::default();
//! // Assuming I2C device address 0x24
//! let expander = PCA9570::new(i2c_bus, 0x24);
//! ```
//! ## Changing mode
//! ```
//!# use pca9570::example::DummyI2CBus;
//!# use pca9570::expander::Mode::{Input, Output};
//!# use pca9570::expander::PCA9570;
//!# use pca9570::expander::PinID::{Pin2, Pin4};
//!#
//!# let i2c_bus = DummyI2CBus::default();
//!# let mut  expander = PCA9570::new(i2c_bus, 0x24);
//!#
//! // Switch Pin1 to input mode
//! expander.set_mode(Pin1, Input).unwrap();
//!
//! // Switch Pin2 to output mode
//! expander.set_mode(Pin2, Output).unwrap();
//! ```
//! ## Reading input state
//! ```
//!# use pca9570::example::DummyI2CBus;
//!# use pca9570::expander::PCA9570;
//!# use pca9570::expander::PinID::Pin1;
//!#
//!# let i2c_bus = DummyI2CBus::default();
//!# let mut  expander = PCA9570::new(i2c_bus, 0x24);
//!#
//! expander.refresh_input_state().unwrap();
//! let is_high = expander.is_pin_input_high(Pin1);
//!
//! assert!(is_high);
//! ```
//! ## Setting output state
//! ```
//!# use pca9570::example::DummyI2CBus;
//!# use pca9570::expander::Mode::Output;
//!# use pca9570::expander::PCA9570;
//!# use pca9570::expander::PinID::Pin1;
//!#
//!# let i2c_bus = DummyI2CBus::default();
//!# let mut  expander = PCA9570::new(i2c_bus, 0x24);
//!#
//! expander.set_mode(Pin1, Output);
//!
//! expander.set_state(Pin1, true);
//! expander.write_output_state().unwrap();
//!
//! let is_high = expander.is_pin_output_high(Pin1);
//! assert!(is_high);
//! ```
//! ## Invert input polarity
//! PCA9570 has built-in hardware support for inverting input state. See [datasheet](<https://www.ti.com/lit/ds/symlink/pca9570.pdf?ts=1649342250975>)
//! for more details.
//! ```
//!# use pca9570::example::DummyI2CBus;
//!# use pca9570::expander::PCA9570;
//!# use pca9570::expander::PinID::{Pin1, Pin3};
//!#
//!# let i2c_bus = DummyI2CBus::default();
//!# let mut  expander = PCA9570::new(i2c_bus, 0x24);
//!#
//! expander.reverse_polarity(Pin3, true).unwrap();
//! ```

#[cfg(feature = "cortex-m")]
use crate::guard::CsMutexGuard;
use crate::guard::LockFreeGuard;
#[cfg(feature = "spin")]
use crate::guard::SpinGuard;
use crate::pins::Pins;
#[cfg(feature = "alloc")]
use alloc::string::{String, ToString};
use bitmaps::Bitmap;
use core::cell::RefCell;
use core::fmt::{Debug, Formatter};
#[cfg(feature = "cortex-m")]
use cortex_m::interrupt::Mutex as CsMutex;
use embedded_hal::blocking::i2c::{Read, SevenBitAddress, Write};
#[cfg(feature = "spin")]
use spin::Mutex as SpinMutex;

/// GPIO pin ID.
#[derive(Copy, Clone)]
pub enum PinID {
    Pin0 = 0,
    Pin1 = 1,
    Pin2 = 2,
    Pin3 = 3,
}

/// GPIO mode
#[derive(PartialEq, Copy, Clone)]
pub enum Mode {
    Output,
    Input,
}

/// Abstraction of [PCA9570](<https://www.ti.com/lit/ds/symlink/pca9570.pdf?ts=1649342250975>) I/O expander
pub struct PCA9570<B>
where
    B: Write<SevenBitAddress> + Read<SevenBitAddress>,
{
    bus: B,

    /// I2C slave address 0x24 + R/!W bit
    address: u8,

    /// First input register
    input: Bitmap<8>,

    /// First output register
    output: Bitmap<8>,

    /// Configuration register
    configuration: Bitmap<8>,
}

/// Wrapped I2C error when refreshing input state
/// Reading input state consists of one write, followed by a read operation
pub enum RefreshInputError<B: Write + Read<u8>> {
    WriteError(<B as Write>::Error),
    ReadError(<B as Read>::Error),
}

impl<B> PCA9570<B>
where
    B: Write<SevenBitAddress> + Read<SevenBitAddress>,
{
    pub fn new(bus: B, address: u8) -> Self {
        let mut expander = Self {
            bus,
            address,
            input: Bitmap::<8>::new(),
            output: Bitmap::<8>::new(),
            configuration: Bitmap::<8>::new(),
        };

        expander.output.invert();
        expander.configuration.invert();

        expander
    }

    // Destroys the driver and returns the I2C bus
    pub fn destroy(self) -> B {
        self.bus
    }

    /// Returns a pins container without using any locks
    /// This is the most efficient way of using individual pins
    /// The downside is, that these pins are neither Send or Sync, so can only be used in single-threaded
    /// and interrupt-free applications
    pub fn pins(&mut self) -> Pins<B, LockFreeGuard<B>> {
        Pins::new(LockFreeGuard::new(RefCell::new(self)))
    }

    /// Returns a pins container using Mutex based on critical sections
    /// Individual pins can be used across threads and interrupts, as long just running on a single core
    #[cfg(feature = "cortex-m")]
    pub fn pins_cs_mutex(&mut self) -> Pins<B, CsMutexGuard<B>> {
        Pins::new(CsMutexGuard::new(CsMutex::new(RefCell::new(self))))
    }

    /// Returns a pins container using a spin mutex
    /// This is safe to use across threads and on multi-core applications
    /// However, this requires a system supporting spin mutexes, which are generally only
    /// available on systems with Atomic CAS
    #[cfg(feature = "spin")]
    pub fn pins_spin_mutex(&mut self) -> Pins<B, SpinGuard<B>> {
        Pins::new(SpinGuard::new(SpinMutex::new(RefCell::new(self))))
    }

    /// Switches the given pin to the input/output mode by adjusting the configuration register
    pub fn set_mode(&mut self, id: PinID, mode: Mode) -> Result<(), <B as Write>::Error> {
        self.configuration.set(id as usize, mode.into());
        self.write_conf()
    }

    /// Switches all pins to output/input mode1
    pub fn set_mode_all(&mut self, mode: Mode) -> Result<(), <B as Write>::Error> {
        let mut bitset = Bitmap::<8>::new();

        if mode == Mode::Input {
            bitset.invert();
        }

        self.configuration = bitset;
        self.write_conf()
    }

    /// Sets the given output state by adjusting the output register
    /// Pin needs to be in OUTPUT mode for correct electrical state
    /// Note: This just updates the internal register, to make the changes effective,
    /// an additional call to `write_output_state()` is needed.
    pub fn set_state(&mut self, id: PinID, is_high: bool) {
        self.output.set(id as usize, is_high);
    }

    /// Sets output state for all pins
    pub fn set_state_all(&mut self, is_high: bool) -> Result<(), <B as Write>::Error> {
        let mut bitset = Bitmap::<8>::new();

        if is_high {
            bitset.invert();
        }

        self.output = bitset;
        self.write_output_state()
    }

    /// Refreshes the input state
    pub fn refresh_input_state(&mut self) -> Result<(), RefreshInputError<B>> {
        self.input = Bitmap::from_value(self.read_input_register()?);
        Ok(())
    }

    /// Returns true if the given pin input is high
    /// Pin needs to be in INPUT mode
    /// This method is using the cached register, for a updated result `refresh_input_state()` needs
    /// to be called beforehand
    pub fn is_pin_input_high(&self, id: PinID) -> bool {
        self.input.get(id as usize)
    }

    /// Returns true if the pins output state is set high
    pub fn is_pin_output_high(&self, id: PinID) -> bool {
        self.output.get(id as usize)
   }

    /// Reads and returns the given input register
    fn read_input_register(&mut self) -> Result<u8, RefreshInputError<B>> {
        self.bus
            .write(self.address, &[])
            .map_err(RefreshInputError::WriteError)?;

        let mut buffer: [u8; 1] = [0x0; 1];
        self.bus.read(self.address, &mut buffer).map_err(RefreshInputError::ReadError)?;

        Ok(buffer[0])
    }

    /// Writes the configuration register
    fn write_conf(&mut self) -> Result<(), <B as Write>::Error> {
        self.bus.write(
            self.address,
            &[*self.configuration.as_value()],
        )
    }

    /// Writes the output register
    pub fn write_output_state(&mut self) -> Result<(), <B as Write>::Error> {
        self.bus.write(self.address, &[*self.output.as_value()])
    }

}

impl From<Mode> for bool {
    fn from(mode: Mode) -> Self {
        match mode {
            Mode::Output => false,
            Mode::Input => true,
        }
    }
}

impl<B: Read<u8> + Write> Debug for RefreshInputError<B> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            RefreshInputError::WriteError(_) => f.write_str("RefreshInputError::WriteError"),
            RefreshInputError::ReadError(_) => f.write_str("RefreshInputError::ReadError"),
        }
    }
}

#[cfg(feature = "alloc")]
impl<B: Read<u8> + Write> ToString for RefreshInputError<B> {
    fn to_string(&self) -> String {
        match self {
            RefreshInputError::WriteError(_) => "WriteError".to_string(),
            RefreshInputError::ReadError(_) => "ReadError".to_string(),
        }
    }
}
