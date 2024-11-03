# embedded-ads1220

Embedded Driver for the Texas Instruments ADS1220 ADC.

Platform agnostic using [embedded-hal](https://crates.io/crates/embedded-hal).

## Example

Run the example [Embassy](https://embassy.dev/) bootstrap code for ST Nucleo SMT32F429ZI using the following:

```shell
# cd examples/embassy-stm32
cargo run --release --bin spi_bootstrap
```

## Credits

This crate is based on the [Protocentral ADS1220](https://github.com/Protocentral/Protocentral_ADS1220) library for Arduino.
