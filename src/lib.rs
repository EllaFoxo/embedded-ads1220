#![no_std]
#![no_main]
use embedded_hal::digital::{ErrorType, InputPin};
use embedded_hal::spi::SpiDevice;

#[cfg(feature = "defmt")]
use defmt::debug;
use embedded_hal::delay::DelayNs;

pub const BYTES_PER_SAMPLE: u8 = 4;

// ADS1220 SPI Commands
pub const SPI_MASTER_DUMMY: u8 = 0xFF;
pub const RESET: u8 = 0x06;
pub const START: u8 = 0x08;
pub const WREG: u8 = 0x40;
pub const RREG: u8 = 0x20;

// Config registers
pub const CONFIG_REG0_ADDRESS: u8 = 0x00;
pub const CONFIG_REG1_ADDRESS: u8 = 0x01;
pub const CONFIG_REG2_ADDRESS: u8 = 0x02;
pub const CONFIG_REG3_ADDRESS: u8 = 0x03;

pub const REG_CONFIG3_IDAC1_ROUTING_MASK: u8 = 0xE0;
pub const REG_CONFIG3_IDAC2_ROUTING_MASK: u8 = 0x1C;
pub const REG_CONFIG2_VREF_MASK: u8 = 0xC0;
pub const REG_CONFIG2_FIR_MASK: u8 = 0x30;
pub const REG_CONFIG2_IDAC_CURRENT_MASK: u8 = 0x07;
pub const REG_CONFIG1_MODE_MASK: u8 = 0x18;
pub const REG_CONFIG1_DR_MASK: u8 = 0xE0;
pub const REG_CONFIG0_PGA_GAIN_MASK: u8 = 0x0E;
pub const REG_CONFIG0_MUX_MASK: u8 = 0xF0;

pub const IDAC1_DISABLE: u8 = 0x00;
pub const IDAC1_AIN0: u8 = 0x20;
pub const IDAC1_AIN1: u8 = 0x40;
pub const IDAC1_AIN2: u8 = 0x60;
pub const IDAC1_AIN3: u8 = 0x80;
pub const IDAC1_REFP0: u8 = 0xA0;
pub const IDAC1_REFN0: u8 = 0xC0;
pub const IDAC1_RESERVED: u8 = 0xE0;

pub const IDAC2_DISABLE: u8 = 0x00;
pub const IDAC2_AIN0: u8 = 0x04;
pub const IDAC2_AIN1: u8 = 0x08;
pub const IDAC2_AIN2: u8 = 0x60;
pub const IDAC2_AIN3: u8 = 0x0C;
pub const IDAC2_REFP0: u8 = 0x10;
pub const IDAC2_REFN0: u8 = 0x14;
pub const IDAC2_RESERVED: u8 = 0x1C;

pub const IDAC_OFF: u8 = 0x00;
pub const IDAC_10: u8 = 0x01;
pub const IDAC_50: u8 = 0x02;
pub const IDAC_100: u8 = 0x03;
pub const IDAC_250: u8 = 0x04;
pub const IDAC_500: u8 = 0x05;
pub const IDAC_1000: u8 = 0x06;
pub const IDAC_1500: u8 = 0x07;

pub const FIR_OFF: u8 = 0x00;
pub const FIR_5060: u8 = 0x10;
pub const FIR_50HZ: u8 = 0x20;
pub const FIR_60HZ: u8 = 0x30;

pub const VREF_2048: u8 = 0x00;
pub const VREF_REFP0: u8 = 0x40;
pub const VREF_AIN0: u8 = 0x80;
pub const VREF_ANALOG: u8 = 0xC0;

pub const MODE_NORMAL: u8 = 0x00;
pub const MODE_DUTY_CYCLE: u8 = 0x08;
pub const MODE_TURBO: u8 = 0x10;
pub const MODE_RESERVED: u8 = 0x18;

pub const DR_20SPS: u8 = 0x00;
pub const DR_45SPS: u8 = 0x02;
pub const DR_90SPS: u8 = 0x04;
pub const DR_175SPS: u8 = 0x06;
pub const DR_330SPS: u8 = 0x08;
pub const DR_600SPS: u8 = 0xA0;
pub const DR_1000SPS: u8 = 0xC0;

pub const PGA_GAIN_1: u8 = 0x00;
pub const PGA_GAIN_2: u8 = 0x02;
pub const PGA_GAIN_4: u8 = 0x04;
pub const PGA_GAIN_8: u8 = 0x06;
pub const PGA_GAIN_16: u8 = 0x08;
pub const PGA_GAIN_32: u8 = 0x0A;
pub const PGA_GAIN_64: u8 = 0x0C;
pub const PGA_GAIN_128: u8 = 0x0E;

pub const MUX_AIN0_AIN1: u8 = 0x00;
pub const MUX_AIN0_AIN2: u8 = 0x10;
pub const MUX_AIN0_AIN3: u8 = 0x20;
pub const MUX_AIN1_AIN2: u8 = 0x30;
pub const MUX_AIN1_AIN3: u8 = 0x40;
pub const MUX_AIN2_AIN3: u8 = 0x50;
pub const MUX_AIN1_AIN0: u8 = 0x60;
pub const MUX_AIN3_AIN2: u8 = 0x70;
pub const MUX_AIN0_AVSS: u8 = 0x80;
pub const MUX_AIN1_AVSS: u8 = 0x90;
pub const MUX_AIN2_AVSS: u8 = 0xA0;
pub const MUX_AIN3_AVSS: u8 = 0xB0;

pub const MUX_SE_CH0: u8 = 0x80;
pub const MUX_SE_CH1: u8 = 0x90;
pub const MUX_SE_CH2: u8 = 0xA0;
pub const MUX_SE_CH3: u8 = 0xB0;

pub const VREF_MASK: u8 = (1 << 6) | (1 << 7);
pub const VREF_INT: u8 = 0 << 6;
pub const VREF_EXT: u8 = 1 << 6;

pub struct ADS1220<SPI, DR> {
    spi: SPI,
    dr: DR,
    m_config_reg0: u8,
    m_config_reg1: u8,
    m_config_reg2: u8,
    m_config_reg3: u8,
}

#[derive(Copy, Clone, Debug)]
pub enum ADS1220Error<SPI, DR> {
    Spi(SPI),
    Dr(DR),
    // Add other errors for your driver here.
}

impl<SPI, DR> ADS1220<SPI, DR>
where
    SPI: SpiDevice,
    DR: InputPin,
{
    pub fn new(
        spi: SPI,
        dr: DR,
    ) -> Self {
        ADS1220 {
            spi,
            dr,
            m_config_reg0: 0x00,
            m_config_reg1: 0x00,
            m_config_reg2: 0x00,
            m_config_reg3: 0x00,
        }
    }

    pub fn write_register(&mut self, address: u8, value: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.spi.write(&[WREG | (address << 2)]).map_err(ADS1220Error::Spi)?;
        self.spi.write(&mut [value]).map_err(ADS1220Error::Spi)?;
        Ok(())
    }

    pub fn read_register(&mut self, address: u8) -> Result<u8, ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        let mut result: [u8; 1] = [0x00];
        self.spi.write(&[RREG | (address << 2)]).map_err(ADS1220Error::Spi)?;
        self.spi.write(&[SPI_MASTER_DUMMY]).map_err(ADS1220Error::Spi)?;
        self.spi.read(&mut result).map_err(ADS1220Error::Spi)?;
        Ok(result[0])
    }

    pub fn begin(&mut self, delay: &mut dyn DelayNs) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.reset()?;
        delay.delay_us(50);

        self.m_config_reg0 = 0x00; // Default settings: AINP=AIN0, AINN=AIN1, Gain 1, PGA enabled
        self.m_config_reg1 = 0x04; // Default settings: DR=20 SPS, Mode=Normal, Conv mode=continuous, Temp Sensor disabled, Current Source off
        self.m_config_reg2 = 0x10; // Default settings: Vref internal, 50/60Hz rejection, power open, IDAC off
        self.m_config_reg3 = 0x00; //  Default settings: IDAC1 disabled, IDAC2 disabled, DRDY pin only

        self.write_register(CONFIG_REG0_ADDRESS, self.m_config_reg0)?;
        self.write_register(CONFIG_REG1_ADDRESS, self.m_config_reg1)?;
        self.write_register(CONFIG_REG2_ADDRESS, self.m_config_reg2)?;
        self.write_register(CONFIG_REG3_ADDRESS, self.m_config_reg3)?;
        Ok(())
    }

    #[cfg(feature = "defmt")]
    pub fn print_register_values(&mut self) {
        let config_reg0 = self.read_register(CONFIG_REG0_ADDRESS).unwrap();
        let config_reg1 = self.read_register(CONFIG_REG1_ADDRESS).unwrap();
        let config_reg2 = self.read_register(CONFIG_REG2_ADDRESS).unwrap();
        let config_reg3 = self.read_register(CONFIG_REG3_ADDRESS).unwrap();

        debug!("Config_Reg : ");
        debug!("{:#04X}", config_reg0);
        debug!("{:#04X}", config_reg1);
        debug!("{:#04X}", config_reg2);
        debug!("{:#04X}", config_reg3);
    }

    pub fn spi_command(&mut self, data: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.spi.write(&mut [data]).map_err(ADS1220Error::Spi)?;
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.spi_command(RESET)
    }

    pub fn start_conv(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.spi_command(START)
    }

    pub fn select_mux_channels(&mut self, channels_conf: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg0 &= !REG_CONFIG0_MUX_MASK;
        self.m_config_reg0 |= channels_conf;
        self.write_register(CONFIG_REG0_ADDRESS, self.m_config_reg0)
    }

    pub fn set_pga_gain(&mut self, pga_gain: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg0 &= !REG_CONFIG0_PGA_GAIN_MASK;
        self.m_config_reg0 |= pga_gain;
        self.write_register(CONFIG_REG0_ADDRESS, self.m_config_reg0)
    }

    pub fn pga_on(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg0 &= !(1 << (0));
        self.write_register(CONFIG_REG0_ADDRESS, self.m_config_reg0)
    }

    pub fn pga_off(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg0 |= !(1 << (0));
        self.write_register(CONFIG_REG0_ADDRESS, self.m_config_reg0)
    }

    pub fn set_data_rate(&mut self, data_rate: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg1 &= !REG_CONFIG1_DR_MASK;
        self.m_config_reg1 |= data_rate;
        self.write_register(CONFIG_REG1_ADDRESS, self.m_config_reg1)
    }

    pub fn set_operation_mode(&mut self, mode: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg1 &= !REG_CONFIG1_MODE_MASK;
        self.m_config_reg1 |= mode;
        self.write_register(CONFIG_REG1_ADDRESS, self.m_config_reg1)
    }

    pub fn set_conv_mode_single_shot(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg1 |= !(1 << (2));
        self.write_register(CONFIG_REG1_ADDRESS, self.m_config_reg1)
    }

    pub fn set_conv_mode_continuous(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg1 |= 1 << (2);
        self.write_register(CONFIG_REG1_ADDRESS, self.m_config_reg1)
    }

    pub fn temp_sensor_mode_disable(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg1 &= !(1 << (1));
        self.write_register(CONFIG_REG1_ADDRESS, self.m_config_reg1)
    }

    pub fn temp_sensor_mode_enable(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg1 |= 1 << (1);
        self.write_register(CONFIG_REG1_ADDRESS, self.m_config_reg1)
    }

    pub fn current_sources_off(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg1 &= !(1 << (0));
        self.write_register(CONFIG_REG1_ADDRESS, self.m_config_reg1)
    }

    pub fn current_sources_on(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg1 |= 1 << (0);
        self.write_register(CONFIG_REG1_ADDRESS, self.m_config_reg1)
    }

    pub fn set_vref(&mut self, vref: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg2 &= !REG_CONFIG2_VREF_MASK;
        self.m_config_reg2 |= vref;
        self.write_register(CONFIG_REG2_ADDRESS, self.m_config_reg2)
    }

    pub fn set_fir_filter(&mut self, filter: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg2 &= !REG_CONFIG2_FIR_MASK;
        self.m_config_reg2 |= filter;
        self.write_register(CONFIG_REG2_ADDRESS, self.m_config_reg2)
    }

    pub fn low_side_switch_open(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg2 &= !(1 << (3));
        self.write_register(CONFIG_REG2_ADDRESS, self.m_config_reg2)
    }

    pub fn low_side_switch_closed(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg2 |= 1 << (3);
        self.write_register(CONFIG_REG2_ADDRESS, self.m_config_reg2)
    }

    pub fn set_idac_current(&mut self, idac_current: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg2 &= !REG_CONFIG2_IDAC_CURRENT_MASK;
        self.m_config_reg2 |= idac_current;
        self.write_register(CONFIG_REG2_ADDRESS, self.m_config_reg2)
    }

    pub fn set_idac1_route(&mut self, idac1_routing: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg3 &= !REG_CONFIG3_IDAC1_ROUTING_MASK;
        self.m_config_reg3 |= idac1_routing;
        self.write_register(CONFIG_REG3_ADDRESS, self.m_config_reg3)
    }

    pub fn set_idac2_route(&mut self, idac2_routing: u8) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg3 &= !REG_CONFIG3_IDAC2_ROUTING_MASK;
        self.m_config_reg3 |= idac2_routing;
        self.write_register(CONFIG_REG3_ADDRESS, self.m_config_reg3)
    }

    pub fn set_drdy_mode_default(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg3 &= !(1 << (3));
        self.write_register(CONFIG_REG3_ADDRESS, self.m_config_reg3)
    }

    pub fn set_drdy_mode_dout(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg3 |= 1 << (3);
        self.write_register(CONFIG_REG3_ADDRESS, self.m_config_reg3)
    }

    pub fn set_internal_reference(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg2 &= !VREF_MASK;
        self.m_config_reg2 |= VREF_INT;
        self.write_register(CONFIG_REG2_ADDRESS, self.m_config_reg2)
    }

    pub fn set_external_reference(&mut self) -> Result<(), ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg2 &= !VREF_MASK;
        self.m_config_reg2 |= VREF_EXT;
        self.write_register(CONFIG_REG2_ADDRESS, self.m_config_reg2)
    }

    pub fn get_config_reg(&mut self) -> Result<[u8; 4], ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.m_config_reg0 = self.read_register(CONFIG_REG0_ADDRESS)?;
        self.m_config_reg1 = self.read_register(CONFIG_REG1_ADDRESS)?;
        self.m_config_reg2 = self.read_register(CONFIG_REG2_ADDRESS)?;
        self.m_config_reg3 = self.read_register(CONFIG_REG3_ADDRESS)?;

        Ok([
            self.m_config_reg0,
            self.m_config_reg1,
            self.m_config_reg2,
            self.m_config_reg3
        ])
    }

    pub fn wait_for_data(&mut self, delay: &mut dyn DelayNs, mut timeout_ms: u64) -> bool {
        while self.dr.is_high().unwrap() {
            if timeout_ms > 0 {
                delay.delay_ms(1);
                timeout_ms -= 1;
            } else {
                return false;
            }
        }
        true
    }

    pub fn read_data(&mut self) -> Result<[u8; 3], ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        let mut buf: [u8; 3] = [0x00; 3];
        self.spi.read(&mut buf).map_err(ADS1220Error::Spi)?;
        Ok(buf)
    }

    pub fn data_to_int(&mut self, data: [u8; 3]) -> i32 {
        let mut result: i32;
        result = data[0] as i32;
        result = (result << 8) | data[1] as i32;
        result = (result << 8) | data[2] as i32;

        if (data[0] & (1 << 7)) == 0 {
            result |= 0xFF000000u32 as i32;
        }
        result
    }

    pub fn read_wait_for_data(&mut self, delay: &mut dyn DelayNs) -> Result<i32, ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        if !self.wait_for_data(delay, 60) {
            return Ok(0);
        }
        self.read_data().map(|t| self.data_to_int(t))
    }

    pub fn read_data_samples(&mut self) -> Result<i32, ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        let mut buf: [u8; 3] = [0x00; 3];
        let result: i32;
        let mut bit24: i32;

        self.spi.read(&mut buf).map_err(ADS1220Error::Spi)?;

        bit24 = buf[0] as i32;
        bit24 = (bit24 << 8) | buf[1] as i32;
        bit24 = (bit24 << 8) | buf[2] as i32;
        bit24 = bit24 << 8;
        result = bit24 >> 8;
        Ok(result)
    }

    pub fn read_single_shot(&mut self) -> Result<i32, ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.start_conv()?;
        self.read_data().map(|t| self.data_to_int(t))
    }

    pub fn read_single_shot_wait_for_data(&mut self, delay: &mut dyn DelayNs) -> Result<i32, ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.start_conv()?;
        self.read_wait_for_data(delay)
    }

    pub fn read_single_shot_single_ended(&mut self, channels_conf: u8) -> Result<i32, ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.select_mux_channels(channels_conf)?;
        self.read_single_shot()
    }

    pub fn read_single_shot_single_ended_wait_for_data(&mut self, delay: &mut dyn DelayNs, channels_conf: u8) -> Result<i32, ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.select_mux_channels(channels_conf)?;
        self.read_single_shot_wait_for_data(delay)
    }

    pub fn data_ready(&mut self) -> Result<bool, ADS1220Error<SPI::Error, <DR as ErrorType>::Error>> {
        self.dr.is_low().map_err(ADS1220Error::Dr)
    }

    #[cfg(feature = "defmt")]
    pub fn status(&mut self) {
        // let current_time: Instant = Instant::now();
        let ready = self.data_ready().unwrap();
        let pending = if ready { BYTES_PER_SAMPLE } else { 0 };
        debug!("ads1220 status: ready={}, pending_bytes={:#X}", ready, pending)
    }
}
