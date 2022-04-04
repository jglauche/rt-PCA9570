#![cfg_attr(not(test), no_std)]
#![cfg_attr(feature = "strict", deny(warnings))]

pub mod expander;

#[cfg(test)]
mod mocks;
#[cfg(test)]
mod tests;
