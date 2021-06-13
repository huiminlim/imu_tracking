/*
    bmi160.c

    Created: 21/12/2020 11:44:27 PM
    Author: user
*/

#include <delay.h>
#include <spi.h>
#include <bmi160.h>


/** Initializes the sensor
	@returns BMI160 sensor ID to verify if connection is successful
*/
uint8_t bmi160_init(void) {
    // Initialization of sensor
    /* Issue a soft-reset to bring the device into a clean state */
    reg_write(BMI160_RA_CMD, BMI160_CMD_SOFT_RESET);
    delay_ms(10);

    /* Issue a dummy-read to force the device into SPI comms mode */
    reg_read(0x7F);
    delay_ms(10);

    /* Power up the accelerometer */
    reg_write(BMI160_RA_CMD, BMI160_CMD_ACC_MODE_NORMAL);
    delay_ms(10);

    /* Wait for power-up to complete */
    while (0x1 != reg_read_bits(BMI160_RA_PMU_STATUS,
                                BMI160_ACC_PMU_STATUS_BIT,
                                BMI160_ACC_PMU_STATUS_LEN)) {
        delay_ms(10);
    }

    /* Power up the gyroscope */
    reg_write(BMI160_RA_CMD, BMI160_CMD_GYR_MODE_NORMAL);
    delay_ms(1);

    /* Wait for power-up to complete */
    while (0x1 != reg_read_bits(BMI160_RA_PMU_STATUS,
                                BMI160_GYR_PMU_STATUS_BIT,
                                BMI160_GYR_PMU_STATUS_LEN)) {
        delay_ms(10);
    }

    bmi160_set_full_scale_gyro_range(BMI160_GYRO_RANGE_250);
    bmi160_set_full_scale_accel_range(BMI160_ACCEL_RANGE_2G);

    /* Only PIN1 interrupts currently supported - map all interrupts to PIN1 */
    reg_write(BMI160_RA_INT_MAP_0, 0xFF);
    reg_write(BMI160_RA_INT_MAP_1, 0xF0);
    reg_write(BMI160_RA_INT_MAP_2, 0x00);

    return reg_read(BMI160_RA_CHIP_ID);
}

/** Check if SPI connection to BMI160 works.
*/
uint8_t bmi160_check_connection(void) {
    return (reg_read(BMI160_RA_CHIP_ID) == BMI160_CHIP_ID);
}

/** Set full-scale gyroscope range.
    @param range New full-scale gyroscope range value
*/
void bmi160_set_full_scale_gyro_range(uint8_t range) {
    reg_write_bits(BMI160_RA_GYRO_RANGE, range,
                   BMI160_GYRO_RANGE_SEL_BIT,
                   BMI160_GYRO_RANGE_SEL_LEN);
}

/** Set full-scale accelerometer range.
    @param range New full-scale accelerometer range setting
*/
void bmi160_set_full_scale_accel_range(uint8_t range) {
    reg_write_bits(BMI160_RA_ACCEL_RANGE, range,
                   BMI160_ACCEL_RANGE_SEL_BIT,
                   BMI160_ACCEL_RANGE_SEL_LEN);
}

/** Get full-scale gyroscope range.
    The gyr_range parameter allows setting the full-scale range of the gyro sensors,
    as described in the table below.

    4 = +/-  125 degrees/sec
    3 = +/-  250 degrees/sec
    2 = +/-  500 degrees/sec
    1 = +/- 1000 degrees/sec
    0 = +/- 2000 degrees/sec

    @return Current full-scale gyroscope range setting
*/
uint16_t bmi160_get_full_scale_gyro_range() {
    uint8_t ret = reg_read_bits(BMI160_RA_GYRO_RANGE,
                                BMI160_GYRO_RANGE_SEL_BIT,
                                BMI160_GYRO_RANGE_SEL_LEN);

    if (ret == 4) {
        return 125;
    }
    else if (ret == 3) {
        return 250;
    }
    else if (ret == 2) {
        return 500;
    }
    else if (ret == 1) {
        return 1000;
    }
    else {
        return 2000;
    }
}

/** Get full-scale accelerometer range.
    The FS_SEL parameter allows setting the full-scale range of the accelerometer
    sensors, as described in the table below.

    3 = +/- 2g
    5 = +/- 4g
    8 = +/- 8g
    12 = +/- 16g

    @return Current full-scale accelerometer range setting
*/
uint8_t bmi160_get_full_scale_accel_range(void) {
    uint8_t ret =  reg_read_bits(BMI160_RA_ACCEL_RANGE,
                                 BMI160_ACCEL_RANGE_SEL_BIT,
                                 BMI160_ACCEL_RANGE_SEL_LEN);

    if (ret == 3) {
        return 2;
    }
    else if (ret == 5) {
        return 4;
    }
    else if (ret == 8) {
        return 8;
    }
    else {
        return 16;
    }
}

/** Sets the gyroscope range of the BMI160
*/
void bmi160_set_gyro_range(uint16_t range) {
    uint8_t bmi_range;

    if (range >= 2000) {
        bmi_range = BMI160_GYRO_RANGE_2000;
    }
    else if (range >= 1000) {
        bmi_range = BMI160_GYRO_RANGE_1000;
    }
    else if (range >= 500) {
        bmi_range = BMI160_GYRO_RANGE_500;
    }
    else if (range >= 250) {
        bmi_range = BMI160_GYRO_RANGE_250;
    }
    else {
        bmi_range = BMI160_GYRO_RANGE_125;
    }

    bmi160_set_full_scale_gyro_range(bmi_range);
}

/** Read the gyroscope value
	@param x pointer reference value of memory location to store x gyroscope value
	@param y pointer reference value of memory location to store y gyroscope value
	@param z pointer reference value of memory location to store z gyroscope value
*/
void bmi160_read_gyroscope(int16_t *x, int16_t *y, int16_t *z) {
    int16_t sx = 0, sy = 0, sz = 0;

    bmi160_get_rotation(&sx, &sy, &sz);

    *x = (int16_t) sx;
    *y = (int16_t) sy;
    *z = (int16_t) sz;
}

/** Get 3-axis gyroscope readings.
    These gyroscope measurement registers, along with the accelerometer
    measurement registers, temperature measurement registers, and external sensor
    data registers, are composed of two sets of registers: an internal register
    set and a user-facing read register set.

    Each 16-bit gyroscope measurement has a full scale configured by
    @setFullScaleGyroRange(). For each full scale setting, the gyroscopes'
    sensitivity per LSB is shown in the table below:

    Full Scale Range   | LSB Sensitivity
    -------------------+----------------
    +/- 125  degrees/s | 262.4 LSB/deg/s
    +/- 250  degrees/s | 131.2 LSB/deg/s
    +/- 500  degrees/s | 65.5  LSB/deg/s
    +/- 1000 degrees/s | 32.8  LSB/deg/s
    +/- 2000 degrees/s | 16.4  LSB/deg/s

    @param x 16-bit signed integer container for X-axis rotation
    @param y 16-bit signed integer container for Y-axis rotation
    @param z 16-bit signed integer container for Z-axis rotation
*/
void bmi160_get_rotation(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];

    // Send address to read from and read first byte
    buffer[0] = read8(BMI160_RA_GYRO_X_L);
    buffer[1] = read8(BMI160_RA_GYRO_X_H);
    buffer[2] = read8(BMI160_RA_GYRO_Y_L);
    buffer[3] = read8(BMI160_RA_GYRO_Y_H);
    buffer[4] = read8(BMI160_RA_GYRO_Z_L);
    buffer[5] = read8(BMI160_RA_GYRO_Z_H);

    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

/** Read the accelerometer value
	@param x pointer reference value of memory location to store x accelerometer value
	@param y pointer reference value of memory location to store y accelerometer value
	@param z pointer reference value of memory location to store z accelerometer value
*/
void bmi160_read_accelerometer(int16_t *x, int16_t *y, int16_t *z) {
    int16_t sx, sy, sz;

    bmi160_get_acceleration(&sx, &sy, &sz);

    *x = (int16_t) sx;
    *y = (int16_t) sy;
    *z = (int16_t) sz;
}

/** Get 3-axis accelerometer readings.
    These registers store the most recent accelerometer measurements.
    Accelerometer measurements are written to these registers at the Output Data Rate
    as configured by @see getAccelRate()

    Each 16-bit accelerometer measurement has a full scale configured by
    @setFullScaleAccelRange. For each full scale setting, the accelerometers'
    sensitivity per LSB is shown in the table below:

    Full Scale Range | LSB Sensitivity
    -----------------+----------------
    +/- 2g           | 8192 LSB/mg
    +/- 4g           | 4096 LSB/mg
    +/- 8g           | 2048 LSB/mg
    +/- 16g          | 1024 LSB/mg

    @param x 16-bit signed integer container for X-axis acceleration
    @param y 16-bit signed integer container for Y-axis acceleration
    @param z 16-bit signed integer container for Z-axis acceleration
*/
void bmi160_get_acceleration(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];

    // Send address to read from and read first byte
    buffer[0] = read8(BMI160_RA_ACCEL_X_L);
    buffer[1] = read8(BMI160_RA_ACCEL_X_H);
    buffer[2] = read8(BMI160_RA_ACCEL_Y_L);
    buffer[3] = read8(BMI160_RA_ACCEL_Y_H);
    buffer[4] = read8(BMI160_RA_ACCEL_Z_L);
    buffer[5] = read8(BMI160_RA_ACCEL_Z_H);

    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

/************************************************************************/
/* HELPER FUNCTIONS												        */
/************************************************************************/
/*!
     @brief  Reads from a given register address
	 Wrapper function to read register from sensor, uses read8() function
*/
uint8_t reg_read (uint8_t reg) {
    uint8_t ret = read8(reg);
    return ret;
}

/*!
     @brief  Writes to a given register address
	 Wrapper function to write to register from sensor, uses write8() function
*/
void reg_write(uint8_t reg, uint8_t data) {
    write8(reg, data);
}

/*!
     @brief  Writes to a given register address specific bits at some position
	 Wrapper function to write to register from sensor, uses reg_write()/write8() function
*/
void reg_write_bits(uint8_t reg, uint8_t data, uint8_t pos, uint8_t len) {
    uint8_t b = reg_read(reg);
    uint8_t mask = ((1 << len) - 1) << pos;
    data <<= pos;									// shift data into correct position
    data &= mask;									// zero all non-important bits in data
    b &= ~(mask);									// zero all important bits in existing byte
    b |= data;										// combine data with existing byte
    reg_write(reg, b);
}

/*!
     @brief  Read a given register address specific bits at some position
	 Wrapper function to read register from sensor, uses reg_read()/read8() function
*/
uint8_t reg_read_bits(uint8_t reg, uint8_t pos, uint8_t len) {
    uint8_t b = reg_read(reg);
    uint8_t mask = (1 << len) - 1;
    b >>= pos;
    b &= mask;
    return b;
}

/*
    This function reads 8 bits from sensor
    With a given reg
*/
uint8_t read8(uint8_t reg) {
    // SS set to low - select slave
    ioport_set_pin_low(SPI_HARDWARE_SS);

    // read, bit 7 set to 1
    spixfer(reg | (1 << BMI160_SPI_READ_BIT));
    uint8_t value = spixfer(0);

    // SS set to high - de-select slave
    ioport_set_pin_high(SPI_HARDWARE_SS);

    return value;
}

/*
    This function writes 8 bits to sensor
    With a given reg
*/
void write8 (uint8_t reg, uint8_t value) {
    // SS set to low - select slave
    ioport_set_pin_low(SPI_HARDWARE_SS);

    // read, bit 7 set to 0
    spixfer(reg);
    spixfer(value);

    // SS set to high - de-select slave
    ioport_set_pin_high(SPI_HARDWARE_SS);
}


/*
    This function transfers 8 bits via SPI
*/
uint8_t spixfer(uint8_t x) {
    // Hardware SPI transfer used
    // Write to MOSI pin and receive on MISO pin
    SPDR = x;

    // NOP to introduce delay to prevent wait
    // Loop form iterating when running at the maximum speed
    // This gives about 10% more speed,
    // even if it seems counter-intuitive at lower speeds it is unnoticed.
    asm volatile("nop");

    while (!(SPSR & (1 << SPIF))) ; // wait

    return SPDR;
}

