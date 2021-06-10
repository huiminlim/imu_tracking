/*
    bmi160.c

    Created: 21/12/2020 11:44:27 PM
    Author: user
*/

#include <spi.h>
#include <bmi160.h>
#include <uart.h>
#include <delay.h>

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


    set_full_scale_gyro_range(BMI160_GYRO_RANGE_250);
    set_full_scale_accel_range(BMI160_ACCEL_RANGE_2G);

    set_gyro_range(250);


    /* Only PIN1 interrupts currently supported - map all interrupts to PIN1 */
    reg_write(BMI160_RA_INT_MAP_0, 0xFF);
    reg_write(BMI160_RA_INT_MAP_1, 0xF0);
    reg_write(BMI160_RA_INT_MAP_2, 0x00);

    return reg_read(BMI160_RA_CHIP_ID);
}

/** Set full-scale gyroscope range.
    @param range New full-scale gyroscope range value
    @see getFullScaleGyroRange()
*/
void set_full_scale_gyro_range(uint8_t range) {
    reg_write_bits(BMI160_RA_GYRO_RANGE, range,
                   BMI160_GYRO_RANGE_SEL_BIT,
                   BMI160_GYRO_RANGE_SEL_LEN);
}

/** Set full-scale accelerometer range.
    @param range New full-scale accelerometer range setting
    @see getFullScaleAccelRange()
    @see BMI160AccelRange
*/
void set_full_scale_accel_range(uint8_t range) {
    reg_write_bits(BMI160_RA_ACCEL_RANGE, range,
                   BMI160_ACCEL_RANGE_SEL_BIT,
                   BMI160_ACCEL_RANGE_SEL_LEN);
}

/** Sets the gyroscope range of the BMI160
	@see getFullScaleAccelRange()
*/
void set_gyro_range(uint16_t range) {
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

    set_full_scale_gyro_range(bmi_range);
}

/** Read the gyroscope value
	@param x pointer reference value of memory location to store x gyroscope value
	@param y pointer reference value of memory location to store y gyroscope value
	@param z pointer reference value of memory location to store z gyroscope value
*/
void read_gyro(int16_t *x, int16_t *y, int16_t *z) {
    int16_t sx = 0, sy = 0, sz = 0;

    get_rotation(&sx, &sy, &sz);

    *x = (int16_t) sx;
    *y = (int16_t) sy;
    *z = (int16_t) sz;
}

/** Get 3-axis gyroscope readings.
    These gyroscope measurement registers, along with the accelerometer
    measurement registers, temperature measurement registers, and external sensor
    data registers, are composed of two sets of registers: an internal register
    set and a user-facing read register set.
    The data within the gyroscope sensors' internal register set is always
    updated at the Output Data Rate. Meanwhile, the user-facing read register set
    duplicates the internal register set's data values whenever the serial
    interface is idle. This guarantees that a burst read of sensor registers will
    read measurements from the same sampling instant. Note that if burst reads
    are not used, the user is responsible for ensuring a set of single byte reads
    correspond to a single sampling instant by checking the Data Ready interrupt.

    Each 16-bit gyroscope measurement has a full scale configured by
    @setFullScaleGyroRange(). For each full scale setting, the gyroscopes'
    sensitivity per LSB is shown in the table below:

    <pre>
    Full Scale Range   | LSB Sensitivity
    -------------------+----------------
    +/- 125  degrees/s | 262.4 LSB/deg/s
    +/- 250  degrees/s | 131.2 LSB/deg/s
    +/- 500  degrees/s | 65.5  LSB/deg/s
    +/- 1000 degrees/s | 32.8  LSB/deg/s
    +/- 2000 degrees/s | 16.4  LSB/deg/s
    </pre>

    @param x 16-bit signed integer container for X-axis rotation
    @param y 16-bit signed integer container for Y-axis rotation
    @param z 16-bit signed integer container for Z-axis rotation
    @see getMotion6()
    @see BMI160_RA_GYRO_X_L
*/
void get_rotation(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];

    // Send address to read from and read first byte
    buffer[0] = read8(BMI160_RA_GYRO_X_L);
    //printf("Byte 1: %d\r\n", buffer[0]);

    buffer[1] = read8(BMI160_RA_GYRO_X_H);
    //printf("Byte 2: %d\r\n", buffer[1]);

    buffer[2] = read8(BMI160_RA_GYRO_Y_L);
    //printf("Byte 3: %d\r\n", buffer[2]);

    buffer[3] = read8(BMI160_RA_GYRO_Y_H);
    //printf("Byte 4: %d\r\n", buffer[3]);

    buffer[4] = read8(BMI160_RA_GYRO_Z_L);
    //printf("Byte 5: %d\r\n", buffer[4]);

    buffer[5] = read8(BMI160_RA_GYRO_Z_H);
    //printf("Byte 6: %d\r\n", buffer[5]);

    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}


void read_accelerometer(int16_t *x, int16_t *y, int16_t *z) {
    int16_t sx, sy, sz;

    get_acceleration(&sx, &sy, &sz);

    *x = (int16_t) sx;
    *y = (int16_t) sy;
    *z = (int16_t) sz;
}

/** Get 3-axis accelerometer readings.
    These registers store the most recent accelerometer measurements.
    Accelerometer measurements are written to these registers at the Output Data Rate
    as configured by @see getAccelRate()

    The accelerometer measurement registers, along with the temperature
    measurement registers, gyroscope measurement registers, and external sensor
    data registers, are composed of two sets of registers: an internal register
    set and a user-facing read register set.

    The data within the accelerometer sensors' internal register set is always
    updated at the Output Data Rate. Meanwhile, the user-facing read register set
    duplicates the internal register set's data values whenever the serial
    interface is idle. This guarantees that a burst read of sensor registers will
    read measurements from the same sampling instant. Note that if burst reads
    are not used, the user is responsible for ensuring a set of single byte reads
    correspond to a single sampling instant by checking the Data Ready interrupt.

    Each 16-bit accelerometer measurement has a full scale configured by
    @setFullScaleAccelRange. For each full scale setting, the accelerometers'
    sensitivity per LSB is shown in the table below:

    <pre>
    Full Scale Range | LSB Sensitivity
    -----------------+----------------
    +/- 2g           | 8192 LSB/mg
    +/- 4g           | 4096 LSB/mg
    +/- 8g           | 2048 LSB/mg
    +/- 16g          | 1024 LSB/mg
    </pre>

    @param x 16-bit signed integer container for X-axis acceleration
    @param y 16-bit signed integer container for Y-axis acceleration
    @param z 16-bit signed integer container for Z-axis acceleration
    @see BMI160_RA_ACCEL_X_L
*/
void get_acceleration(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];

    // Send address to read from and read first byte
    buffer[0] = read8(BMI160_RA_ACCEL_X_L);
    printf("Byte 1: %d\r\n", buffer[0]);

    buffer[1] = read8(BMI160_RA_ACCEL_X_H);
    printf("Byte 2: %d\r\n", buffer[1]);

    buffer[2] = read8(BMI160_RA_ACCEL_Y_L);
    printf("Byte 3: %d\r\n", buffer[2]);

    buffer[3] = read8(BMI160_RA_ACCEL_Y_H);
    printf("Byte 4: %d\r\n", buffer[3]);

    buffer[4] = read8(BMI160_RA_ACCEL_Z_L);
    printf("Byte 5: %d\r\n", buffer[4]);

    buffer[5] = read8(BMI160_RA_ACCEL_Z_H);
    printf("Byte 6: %d\r\n", buffer[5]);

    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

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
    //uint8_t buffer[2];
    //buffer[0] = reg;
    //buffer[1] = data;
    //serial_buffer_transfer(buffer, 2, 0);
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
    ioport_set_pin_low(SPI_GYRO_SS);

    // read, bit 7 set to 1
    spixfer(reg | (1 << BMI160_SPI_READ_BIT));
    uint8_t value = spixfer(0);

    // SS set to high - de-select slave
    ioport_set_pin_high(SPI_GYRO_SS);

    return value;
}


/*
    This function writes 8 bits to sensor
    With a given reg
*/
void write8 (uint8_t reg, uint8_t value) {
    // SS set to low - select slave
    ioport_set_pin_low(SPI_GYRO_SS);

    // read, bit 7 set to 0
    spixfer(reg);
    spixfer(value);

    // SS set to high - de-select slave
    ioport_set_pin_high(SPI_GYRO_SS);
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

