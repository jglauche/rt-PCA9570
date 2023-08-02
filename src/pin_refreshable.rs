use crate::expander::{Mode, PinID, RefreshInputError};
use crate::guard::RefGuard;
use crate::pins::{Input, Output, Pin, PinMode, RefreshMode};
use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::blocking::i2c::{Read, Write};
use embedded_hal::digital::v2::{toggleable, InputPin, IoPin, OutputPin, PinState, StatefulOutputPin};

/// Trait for refreshable pins in output mode
pub trait RefreshableOutputPin {
    type Error;

    /// Updates the output state of all pins
    fn update_all(&self) -> Result<(), Self::Error>;
}

/// Trait for refreshable pins in input mode
pub trait RefreshableInputPin {
    type Error;

    /// Refreshes the input state of all pins
    fn refresh_all(&self) -> Result<(), Self::Error>;
}

impl<'a, B, R> Pin<'a, B, R, Input, RefreshMode>
where
    B: Write + Read,
    R: RefGuard<B>,
{
    pub fn refreshable(expander: &'a R, id: PinID) -> Self {
        Self {
            expander,
            bus: PhantomData,
            id,
            access_mode: PhantomData,
            mode: PhantomData,
        }
    }

    /// Refreshes the input state
    fn refresh(&self) -> Result<(), RefreshInputError<B>> {
        let mut result = Ok(());

        self.expander.access(|expander| {
            result = expander.refresh_input_state();
        });

        result
    }
}

impl<'a, B, R> RefreshableInputPin for Pin<'a, B, R, Input, RefreshMode>
where
    B: Write + Read,
    R: RefGuard<B>,
{
    type Error = RefreshInputError<B>;

    /// Refreshes the input state of all pins
    fn refresh_all(&self) -> Result<(), Self::Error> {
        self.refresh()
    }
}

impl<'a, B, R> RefreshableOutputPin for Pin<'a, B, R, Output, RefreshMode>
where
    B: Write + Read,
    R: RefGuard<B>,
{
    type Error = <B as Write>::Error;

    /// Updates the output state of all pins
    fn update_all(&self) -> Result<(), Self::Error> {
        self.update()
    }
}

impl<'a, B, R> Pin<'a, B, R, Output, RefreshMode>
where
    B: Write + Read,
    R: RefGuard<B>,
{
    /// Writes the output state
    fn update(&self) -> Result<(), <B as Write>::Error> {
        let mut result = Ok(());

        self.expander.access(|expander| {
            result = expander.write_output_state();
        });

        result
    }
}

impl<'a, B, R> InputPin for Pin<'a, B, R, Input, RefreshMode>
where
    B: Write + Read,
    R: RefGuard<B>,
{
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        let mut state = false;

        self.expander.access(|expander| {
            state = expander.is_pin_input_high(self.id);
        });

        Ok(state)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

impl<'a, B, R> OutputPin for Pin<'a, B, R, Output, RefreshMode>
where
    B: Read + Write,
    R: RefGuard<B>,
{
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_state(PinState::Low)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_state(PinState::High)
    }

    fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> {
        self.expander.access(|expander| {
            expander.set_state(self.id, state == PinState::High);
        });

        Ok(())
    }
}

impl<'a, B, R> StatefulOutputPin for Pin<'a, B, R, Output, RefreshMode>
where
    B: Write + Read,
    R: RefGuard<B>,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_pin_output_high())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_pin_output_high())
    }
}

impl<'a, B, R> toggleable::Default for Pin<'a, B, R, Output, RefreshMode>
where
    B: Write + Read,
    R: RefGuard<B>,
{
}

impl<'a, B, M, R> IoPin<Pin<'a, B, R, Input, RefreshMode>, Pin<'a, B, R, Output, RefreshMode>>
    for Pin<'a, B, R, M, RefreshMode>
where
    B: Write + Read,
    R: RefGuard<B>,
    M: PinMode,
{
    type Error = <B as Write>::Error;

    fn into_input_pin(self) -> Result<Pin<'a, B, R, Input, RefreshMode>, Self::Error> {
        self.change_mode(Mode::Input)?;

        Ok(Pin {
            expander: self.expander,
            id: self.id,
            bus: PhantomData,
            mode: PhantomData,
            access_mode: PhantomData,
        })
    }

    fn into_output_pin(self, state: PinState) -> Result<Pin<'a, B, R, Output, RefreshMode>, Self::Error> {
        self.change_mode(Mode::Output)?;

        let mut pin = Pin {
            expander: self.expander,
            id: self.id,
            bus: PhantomData,
            mode: PhantomData,
            access_mode: PhantomData,
        };

        let _ = pin.set_state(state);
        pin.update_all()?;
        Ok(pin)
    }
}
