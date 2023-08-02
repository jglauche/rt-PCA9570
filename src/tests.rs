use crate::expander::Mode::{Input, Output};
use crate::expander::PinID::{Pin0, Pin1, Pin2, Pin3};
use crate::expander::PCA9570;
#[cfg(not(feature = "spin"))]
use crate::guard::LockFreeGuard;
#[cfg(feature = "spin")]
use crate::guard::SpinGuard;
use crate::mocks::{BusMockBuilder, MockI2CBus, WriteError};
use crate::pin_refreshable::{RefreshableInputPin, RefreshableOutputPin};
use crate::pins::Pins;
use alloc::string::ToString;
use embedded_hal::digital::v2::{InputPin, IoPin, OutputPin, PinState, StatefulOutputPin, ToggleableOutputPin};

#[test]
fn test_expander_output_mode() {
    let i2c_bus = BusMockBuilder::new()
        .expect_write(1, &[0b1111_0111])
        .expect_write(1, &[0b1111_0110])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_mode(Pin3, Output).unwrap();
    expander.set_mode(Pin0, Output).unwrap();
}

#[test]
fn test_expander_input_mode() {
    let i2c_bus = BusMockBuilder::new()
        .mock_write(1)
        .expect_write(1, &[0b0000_0100])
        .expect_write(1, &[0b0000_1100])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_mode_all(Output).unwrap();
    expander.set_mode(Pin2, Input).unwrap();
    expander.set_mode(Pin3, Input).unwrap();
}

#[test]
fn test_expander_state_low() {
    let i2c_bus = BusMockBuilder::new()
        .expect_write(1, &[0b1111_1101])
        .expect_write(1, &[0b1111_1011])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_state(Pin1, false);
    expander.write_output_state().unwrap();
    expander.set_state(Pin2, false);
    expander.write_output_state().unwrap();
}

#[test]
fn test_expander_state_high() {
    let i2c_bus = BusMockBuilder::new()
        .mock_write(1)
        .expect_write(1, &[0b0000_1000])
        .expect_write(1, &[0b0000_1001])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_state_all(false).unwrap();
    expander.set_state(Pin3, true);
    expander.write_output_state().unwrap();
    expander.set_state(Pin0, true);
    expander.write_output_state().unwrap();
}

#[test]
fn test_set_mode_all_input() {
    let i2c_bus = BusMockBuilder::new()
        .mock_write(1)
        .expect_write(1, &[0b1111_1111])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_mode_all(Output).unwrap();
    expander.set_mode_all(Input).unwrap();
}

#[test]
fn test_set_mode_all_output() {
    let i2c_bus = BusMockBuilder::new().expect_write(1, &[0b0000_0000]).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_mode_all(Output).unwrap();
}

#[test]
fn test_set_state_all_low() {
    let i2c_bus = BusMockBuilder::new()
        .mock_write(1)
        .expect_write(1, &[0b0000_0000])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_state_all(true).unwrap();
    expander.set_state_all(false).unwrap();
}

#[test]
fn test_set_state_all_high() {
    let i2c_bus = BusMockBuilder::new().expect_write(1, &[0b1111_1111]).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_state_all(true).unwrap();
}

#[test]
fn test_refresh_input_state() {
    let i2c_bus = BusMockBuilder::new()
        .expect_write(1, &[0x00])
        .expect_read(1, 0b0000_0000)
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.refresh_input_state().unwrap();
}

#[test]
fn test_refresh_input_state_write_error() {
    let i2c_bus = BusMockBuilder::new().write_error(0x00).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let result = expander.refresh_input_state();

    assert_eq!("WriteError", result.unwrap_err().to_string());
}

#[test]
fn test_refresh_input_state_read_error() {
    let i2c_bus = BusMockBuilder::new().expect_write(1, &[0x00]).read_error().into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let result = expander.refresh_input_state();

    assert_eq!("ReadError", result.unwrap_err().to_string());
}

#[test]
fn test_is_pin_high() {
    let i2c_bus = BusMockBuilder::new()
        .expect_write(1, &[0x00])
        .expect_read(1, 0b0111_1010)
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.refresh_input_state().unwrap();

    assert!(expander.is_pin_input_high(Pin3));
    assert!(!expander.is_pin_input_high(Pin2));
    assert!(expander.is_pin_input_high(Pin1));
    assert!(!expander.is_pin_input_high(Pin0));
}

#[test]
fn test_regular_pin_input() {
    let i2c_bus = BusMockBuilder::new()
        .expect_write(4, &[0x00])
        .expect_read(2, 0b0000_0100)
        .expect_read(2, 0b0100_0000)
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let pin = pins.get_pin(Pin2);

    assert!(pin.is_high().unwrap());
    assert!(!pin.is_low().unwrap());
    assert!(!pin.is_high().unwrap());
    assert!(pin.is_low().unwrap());
}

#[test]
fn test_regular_pin_input_write_error() {
    let i2c_bus = BusMockBuilder::new().write_error(0x01).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let pin = pins.get_pin(Pin3);

    assert_eq!("WriteError", pin.is_high().unwrap_err().to_string())
}

#[test]
fn test_regular_pin_input_read_error() {
    let i2c_bus = BusMockBuilder::new().mock_write(1).read_error().into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let pin = pins.get_pin(Pin3);

    assert_eq!("ReadError", pin.is_high().unwrap_err().to_string())
}

#[test]
fn test_refreshable_pin_input() {
    let i2c_bus = BusMockBuilder::new()
        .expect_write(2, &[0x00])
        .expect_read(1, 0b0000_0100)
        .expect_read(1, 0b0100_1000)
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);

    let pin02 = pins.get_refreshable_pin(Pin2);
    let pin03 = pins.get_refreshable_pin(Pin3);

    pin02.refresh_all().unwrap();
    assert!(pin02.is_high().unwrap());
    assert!(!pin02.is_low().unwrap());
    assert!(!pin03.is_high().unwrap());
    assert!(pin03.is_low().unwrap());

    pin03.refresh_all().unwrap();
    assert!(!pin02.is_high().unwrap());
    assert!(pin02.is_low().unwrap());
    assert!(pin03.is_high().unwrap());
    assert!(!pin03.is_low().unwrap());
}


#[test]
fn test_refreshable_pin_refresh_all_write_error() {
    let i2c_bus = BusMockBuilder::new()
        .expect_write(1, &[0x0])
        .expect_read(1, 0b0001_0000)
        .write_error(0x1)
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);

    let pin = pins.get_refreshable_pin(Pin0);
    let error = pin.refresh_all().unwrap_err();

    assert_eq!("WriteError", error.to_string());
    assert!(pin.is_low().unwrap());
}

#[test]
fn test_refreshable_pin_refresh_all_read_error() {
    let i2c_bus = BusMockBuilder::new()
        .expect_write(1, &[0x0])
        .expect_read(1, 0b0001_0000)
        .expect_write(1, &[0x1])
        .read_error()
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);

    let pin = pins.get_refreshable_pin(Pin0);
    let error = pin.refresh_all().unwrap_err();

    assert_eq!("ReadError", error.to_string());
    assert!(pin.is_low().unwrap());
}

#[test]
fn test_regular_pin_set_output_state() {
    let i2c_bus = BusMockBuilder::new()
        .mock_write(6) // Mode switch
        .expect_write(1, &[0x03, 0b1111_1011])
        .expect_write(1, &[0x02, 0b1110_1111])
        .expect_write(1, &[0x02, 0b1110_1110])
        .expect_write(1, &[0x02, 0b1111_1110])
        .expect_write(1, &[0x02, 0b1111_1110])
        .expect_write(1, &[0x02, 0b1111_1111])
        .expect_write(1, &[0x03, 0b1111_1111])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let mut pin00 = pins.get_pin(Pin0).into_output_pin(PinState::High).unwrap();
    let mut pin01 = pins.get_pin(Pin1).into_output_pin(PinState::High).unwrap();
    let mut pin03 = pins.get_pin(Pin3).into_output_pin(PinState::High).unwrap();

    pin03.set_low().unwrap();
    assert!(pin03.is_set_low().unwrap());
    assert!(!pin03.is_set_high().unwrap());

    pin01.set_low().unwrap();
    assert!(pin01.is_set_low().unwrap());
    assert!(!pin01.is_set_high().unwrap());

    pin00.set_state(PinState::Low).unwrap();
    assert!(pin00.is_set_low().unwrap());
    assert!(!pin00.is_set_high().unwrap());

    pin01.set_state(PinState::High).unwrap();
    assert!(!pin01.is_set_low().unwrap());
    assert!(pin01.is_set_high().unwrap());

    pin01.set_high().unwrap();
    assert!(!pin01.is_set_low().unwrap());
    assert!(pin01.is_set_high().unwrap());

    pin00.set_high().unwrap();
    assert!(!pin00.is_set_low().unwrap());
    assert!(pin00.is_set_high().unwrap());

    pin03.set_high().unwrap();
    assert!(!pin03.is_set_low().unwrap());
    assert!(pin03.is_set_high().unwrap());
}

#[test]
fn test_regular_pin_set_low_write_error() {
    let i2c_bus = BusMockBuilder::new().mock_write(2).write_error(0x2).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let mut pin = pins.get_pin(Pin0).into_output_pin(PinState::Low).unwrap();

    let result = pin.set_low();
    assert_eq!(WriteError::Error1, result.unwrap_err());
}

#[test]
fn test_regular_pin_set_high_write_error() {
    let i2c_bus = BusMockBuilder::new().mock_write(2).write_error(0x2).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let mut pin = pins.get_pin(Pin0).into_output_pin(PinState::Low).unwrap();

    let result = pin.set_high();
    assert_eq!(WriteError::Error1, result.unwrap_err());
}

#[test]
fn test_regular_pin_set_state_write_error() {
    let i2c_bus = BusMockBuilder::new().mock_write(2).write_error(0x2).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let mut pin = pins.get_pin(Pin0).into_output_pin(PinState::Low).unwrap();

    let result = pin.set_state(PinState::High);
    assert_eq!(WriteError::Error1, result.unwrap_err());
}

#[test]
fn test_refreshable_pin_set_output_state() {
    let i2c_bus = BusMockBuilder::new()
        .mock_write(2) // setting all low
        .mock_write(16) // mode switch
        .expect_write(1, &[0x02, 0b0000_0110]) // Update 0
        .expect_write(1, &[0x03, 0b1110_0000]) // Update Bank 1
        .expect_write(1, &[0x02, 0b0000_0110]) // Update all
        .expect_write(1, &[0x03, 0b1110_0000]) // Update all
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_state_all(false).unwrap();
    expander.set_state_all(false).unwrap();

    let pins = get_pins(&mut expander);
    let mut pin00 = pins.get_refreshable_pin(Pin0).into_output_pin(PinState::Low).unwrap();
    let mut pin01 = pins.get_refreshable_pin(Pin1).into_output_pin(PinState::Low).unwrap();
    let mut pin02 = pins.get_refreshable_pin(Pin2).into_output_pin(PinState::Low).unwrap();
    let mut pin03 = pins.get_refreshable_pin(Pin3).into_output_pin(PinState::Low).unwrap();

    pin00.set_low().unwrap();
    assert!(pin00.is_set_low().unwrap());
    assert!(!pin00.is_set_high().unwrap());

    pin01.set_high().unwrap();
    assert!(!pin01.is_set_low().unwrap());
    assert!(pin01.is_set_high().unwrap());

    pin02.set_high().unwrap();
    assert!(!pin02.is_set_low().unwrap());
    assert!(pin02.is_set_high().unwrap());

    pin03.set_low().unwrap();
    assert!(pin03.is_set_low().unwrap());
    assert!(!pin03.is_set_high().unwrap());

    pin03.update_all().unwrap();
}

#[test]
fn test_regular_pin_into_output_pin() {
    let i2c_bus = BusMockBuilder::new()
        .mock_write(1)
        .expect_write(1, &[0x06, 0b1111_1110])
        .expect_write(1, &[0x02, 0b0000_0001])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_state_all(false).unwrap();
    let pins = get_pins(&mut expander);
    let _pin = pins.get_pin(Pin0).into_output_pin(PinState::High).unwrap();
}

#[test]
fn test_regular_pin_into_input_pin() {
    let i2c_bus = BusMockBuilder::new()
        .mock_write(2)
        .expect_write(1, &[0x06, 0b1111_1111])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);

    let pins = get_pins(&mut expander);
    let _pin = pins
        .get_pin(Pin0)
        .into_output_pin(PinState::High)
        .unwrap()
        .into_input_pin()
        .unwrap();
}

#[test]
fn test_regular_pin_into_output_pin_mode_switch_error() {
    let i2c_bus = BusMockBuilder::new().write_error(0x6).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let result = pins.get_pin(Pin0).into_output_pin(PinState::High);

    assert!(result.is_err())
}

#[test]
fn test_regular_pin_into_output_pin_state_set_error() {
    let i2c_bus = BusMockBuilder::new().mock_write(1).write_error(0x2).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let result = pins.get_pin(Pin0).into_output_pin(PinState::High);

    assert!(result.is_err())
}

#[test]
fn test_regular_pin_into_input_pin_mode_error() {
    let i2c_bus = BusMockBuilder::new().write_error(0x6).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let result = pins.get_pin(Pin0).into_output_pin(PinState::High);

    assert!(result.is_err())
}

#[test]
fn test_refreshable_pin_into_output_pin() {
    let i2c_bus = BusMockBuilder::new()
        .expect_write(1, &[0b1111_1110])
        .expect_write(1, &[0b0000_0001])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    expander.set_state_all(false).unwrap();
    let pins = get_pins(&mut expander);
    let _pin = pins.get_refreshable_pin(Pin0).into_output_pin(PinState::High).unwrap();
}

#[test]
fn test_refreshable_pin_into_input_pin() {
    let i2c_bus = BusMockBuilder::new()
        .mock_write(2)
        .expect_write(1, &[0x06, 0b1111_1111])
        .into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);

    let pins = get_pins(&mut expander);
    let _pin = pins
        .get_refreshable_pin(Pin0)
        .into_output_pin(PinState::High)
        .unwrap()
        .into_input_pin()
        .unwrap();
}

#[test]
fn test_refreshable_pin_into_output_pin_mode_switch_error() {
    let i2c_bus = BusMockBuilder::new().write_error(0x6).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let result = pins.get_refreshable_pin(Pin0).into_output_pin(PinState::High);

    assert!(result.is_err())
}

#[test]
fn test_refreshable_pin_into_output_pin_state_set_error() {
    let i2c_bus = BusMockBuilder::new().mock_write(1).write_error(0x2).into_mock();

    let mut expander = PCA9570::new(i2c_bus, 0x7C);
    let pins = get_pins(&mut expander);
    let result = pins.get_refreshable_pin(Pin0).into_output_pin(PinState::High);

    assert!(result.is_err())
}

/// Testing spin based RefGuard
#[cfg(feature = "spin")]
fn get_pins(expander: &mut PCA9570<MockI2CBus>) -> Pins<MockI2CBus, SpinGuard<MockI2CBus>> {
    expander.pins_spin_mutex()
}

/// Testing lock-free RefGuard
#[cfg(not(feature = "spin"))]
fn get_pins(expander: &mut PCA9570<MockI2CBus>) -> Pins<MockI2CBus, LockFreeGuard<MockI2CBus>> {
    expander.pins()
}
