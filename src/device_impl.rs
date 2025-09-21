use crate::markers::ResolutionSupport;
use crate::{conversion, ic, Address, Config, Error, FaultQueue, Lm75, OsMode, OsPolarity};
use core::marker::PhantomData;
#[cfg(not(feature = "async"))]
use embedded_hal::i2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c;

struct Register;

impl Register {
    const TEMPERATURE: u8 = 0x00;
    const CONFIGURATION: u8 = 0x01;
    const T_HYST: u8 = 0x02;
    const T_OS: u8 = 0x03;
    const T_IDLE: u8 = 0x04;
}

struct BitFlags;

impl BitFlags {
    const SHUTDOWN: u8 = 0b0000_0001;
    const COMP_INT: u8 = 0b0000_0010;
    const OS_POLARITY: u8 = 0b0000_0100;
    const FAULT_QUEUE0: u8 = 0b0000_1000;
    const FAULT_QUEUE1: u8 = 0b0001_0000;
}

impl<I2C, E> Lm75<I2C, ic::Lm75>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Create new instance of the LM75 device.
    pub fn new<A: Into<Address>>(i2c: I2C, address: A) -> Self {
        let a = address.into();
        Lm75 {
            i2c,
            address: a.0,
            config: Config::default(),
            _ic: PhantomData,
        }
    }
}

impl<I2C, IC> Lm75<I2C, IC> {
    /// Destroy driver instance, return I²C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), keep_self),
    async(feature = "async", keep_self)
)]
impl<I2C, IC, E> Lm75<I2C, IC>
where
    I2C: i2c::I2c<Error = E>,
    IC: ResolutionSupport<E>,
{
    /// Enable the sensor (default state).
    pub async fn enable(&mut self) -> Result<(), Error<E>> {
        let config = self.config;
        self.write_config(config.with_low(BitFlags::SHUTDOWN)).await
    }

    /// Disable the sensor (shutdown).
    pub async fn disable(&mut self) -> Result<(), Error<E>> {
        let config = self.config;
        self.write_config(config.with_high(BitFlags::SHUTDOWN))
            .await
    }

    /// Set the fault queue.
    ///
    /// Set the number of consecutive faults that will trigger an OS condition.
    pub async fn set_fault_queue(&mut self, fq: FaultQueue) -> Result<(), Error<E>> {
        let config = self.config;
        match fq {
            FaultQueue::_1 => self.write_config(
                config
                    .with_low(BitFlags::FAULT_QUEUE1)
                    .with_low(BitFlags::FAULT_QUEUE0),
            ),
            FaultQueue::_2 => self.write_config(
                config
                    .with_low(BitFlags::FAULT_QUEUE1)
                    .with_high(BitFlags::FAULT_QUEUE0),
            ),
            FaultQueue::_4 => self.write_config(
                config
                    .with_high(BitFlags::FAULT_QUEUE1)
                    .with_low(BitFlags::FAULT_QUEUE0),
            ),
            FaultQueue::_6 => self.write_config(
                config
                    .with_high(BitFlags::FAULT_QUEUE1)
                    .with_high(BitFlags::FAULT_QUEUE0),
            ),
        }
        .await
    }

    /// Set the OS polarity.
    pub async fn set_os_polarity(&mut self, polarity: OsPolarity) -> Result<(), Error<E>> {
        let config = self.config;
        match polarity {
            OsPolarity::ActiveLow => self.write_config(config.with_low(BitFlags::OS_POLARITY)),
            OsPolarity::ActiveHigh => self.write_config(config.with_high(BitFlags::OS_POLARITY)),
        }
        .await
    }

    /// Set the OS operation mode.
    pub async fn set_os_mode(&mut self, mode: OsMode) -> Result<(), Error<E>> {
        let config = self.config;
        match mode {
            OsMode::Comparator => self.write_config(config.with_low(BitFlags::COMP_INT)),
            OsMode::Interrupt => self.write_config(config.with_high(BitFlags::COMP_INT)),
        }
        .await
    }

    /// Set the OS temperature (celsius).
    #[allow(clippy::manual_range_contains)]
    pub async fn set_os_temperature(&mut self, temperature: f32) -> Result<(), Error<E>> {
        if temperature < -55.0 || temperature > 125.0 {
            return Err(Error::InvalidInputData);
        }
        let (msb, lsb) =
            conversion::convert_temp_to_register(temperature, IC::get_resolution_mask());
        self.i2c
            .write(self.address, &[Register::T_OS, msb, lsb])
            .await
            .map_err(Error::I2C)
    }

    /// Set the hysteresis temperature (celsius).
    #[allow(clippy::manual_range_contains)]
    pub async fn set_hysteresis_temperature(&mut self, temperature: f32) -> Result<(), Error<E>> {
        if temperature < -55.0 || temperature > 125.0 {
            return Err(Error::InvalidInputData);
        }
        let (msb, lsb) =
            conversion::convert_temp_to_register(temperature, IC::get_resolution_mask());
        self.i2c
            .write(self.address, &[Register::T_HYST, msb, lsb])
            .await
            .map_err(Error::I2C)
    }

    /// Read the temperature from the sensor (celsius).
    pub async fn read_temperature(&mut self) -> Result<f32, Error<E>> {
        let mut data = [0; 2];
        self.i2c
            .write_read(self.address, &[Register::TEMPERATURE], &mut data)
            .await
            .map_err(Error::I2C)?;
        Ok(conversion::convert_temp_from_register(
            data[0],
            data[1],
            IC::get_resolution_mask(),
        ))
    }

    /// write configuration to device
    async fn write_config(&mut self, config: Config) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[Register::CONFIGURATION, config.bits])
            .await
            .map_err(Error::I2C)?;
        self.config = config;
        Ok(())
    }
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), keep_self),
    async(feature = "async", keep_self)
)]
impl<I2C, E> Lm75<I2C, ic::Pct2075>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Create new instance of the PCT2075 device.
    pub fn new_pct2075<A: Into<Address>>(i2c: I2C, address: A) -> Self {
        let a = address.into();
        Lm75 {
            i2c,
            address: a.0,
            config: Config::default(),
            _ic: PhantomData,
        }
    }

    /// Set the sensor sample rate period in milliseconds (100ms increments).
    ///
    /// For values outside of the range `[100 - 3100]` or those not a multiple of 100,
    /// `Error::InvalidInputData will be returned
    pub async fn set_sample_rate(&mut self, period: u16) -> Result<(), Error<E>> {
        if period > 3100 || period % 100 != 0 {
            return Err(Error::InvalidInputData);
        }
        let byte = conversion::convert_sample_rate_to_register(period);
        self.i2c
            .write(self.address, &[Register::T_IDLE, byte])
            .await
            .map_err(Error::I2C)
    }

    /// Read the sample rate period from the sensor (ms).
    pub async fn read_sample_rate(&mut self) -> Result<u16, Error<E>> {
        let mut data = [0; 1];
        self.i2c
            .write_read(self.address, &[Register::T_IDLE], &mut data)
            .await
            .map_err(Error::I2C)?;
        Ok(conversion::convert_sample_rate_from_register(data[0]))
    }
}
