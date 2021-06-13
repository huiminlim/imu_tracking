/*
    bmi160.h

    Created: 21/12/2020 11:18:14 PM
    Author: user
*/

#define BMI160_INIT_NO_ERR			1
#define BMI160_INIT_ERR				0

#define BMI160_CHIP_ID				0xD1

/**************************/

#define BMI160_SPI_READ_BIT         7

#define BMI160_RA_CHIP_ID           0x00

#define BMI160_ACC_PMU_STATUS_BIT   4
#define BMI160_ACC_PMU_STATUS_LEN   2
#define BMI160_GYR_PMU_STATUS_BIT   2
#define BMI160_GYR_PMU_STATUS_LEN   2

#define BMI160_RA_PMU_STATUS        0x03

#define BMI160_RA_GYRO_X_L          0x0C
#define BMI160_RA_GYRO_X_H          0x0D
#define BMI160_RA_GYRO_Y_L          0x0E
#define BMI160_RA_GYRO_Y_H          0x0F
#define BMI160_RA_GYRO_Z_L          0x10
#define BMI160_RA_GYRO_Z_H          0x11
#define BMI160_RA_ACCEL_X_L         0x12
#define BMI160_RA_ACCEL_X_H         0x13
#define BMI160_RA_ACCEL_Y_L         0x14
#define BMI160_RA_ACCEL_Y_H         0x15
#define BMI160_RA_ACCEL_Z_L         0x16
#define BMI160_RA_ACCEL_Z_H         0x17

#define BMI160_RA_TEMP_L            0x20
#define BMI160_RA_TEMP_H            0x21

#define BMI160_ACCEL_RATE_SEL_BIT    0
#define BMI160_ACCEL_RATE_SEL_LEN    4

#define BMI160_RA_ACCEL_CONF        0X40
#define BMI160_RA_ACCEL_RANGE       0X41

#define BMI160_GYRO_RATE_SEL_BIT    0
#define BMI160_GYRO_RATE_SEL_LEN    4

#define BMI160_RA_GYRO_CONF         0X42
#define BMI160_RA_GYRO_RANGE        0X43

#define BMI160_RA_INT_LATCH         0x54
#define BMI160_RA_INT_MAP_0         0x55
#define BMI160_RA_INT_MAP_1         0x56
#define BMI160_RA_INT_MAP_2         0x57

#define BMI160_GYR_OFFSET_X_MSB_BIT 0
#define BMI160_GYR_OFFSET_X_MSB_LEN 2
#define BMI160_GYR_OFFSET_Y_MSB_BIT 2
#define BMI160_GYR_OFFSET_Y_MSB_LEN 2
#define BMI160_GYR_OFFSET_Z_MSB_BIT 4
#define BMI160_GYR_OFFSET_Z_MSB_LEN 2
#define BMI160_ACC_OFFSET_EN        6
#define BMI160_GYR_OFFSET_EN        7

#define BMI160_RA_OFFSET_0          0x71
#define BMI160_RA_OFFSET_1          0x72
#define BMI160_RA_OFFSET_2          0x73
#define BMI160_RA_OFFSET_3          0x74
#define BMI160_RA_OFFSET_4          0x75
#define BMI160_RA_OFFSET_5          0x76
#define BMI160_RA_OFFSET_6          0x77

#define BMI160_GYRO_RANGE_SEL_BIT   0
#define BMI160_GYRO_RANGE_SEL_LEN   3

#define BMI160_GYRO_RATE_SEL_BIT    0
#define BMI160_GYRO_RATE_SEL_LEN    4

#define BMI160_GYRO_DLPF_SEL_BIT    4
#define BMI160_GYRO_DLPF_SEL_LEN    2

#define BMI160_ACCEL_DLPF_SEL_BIT   4
#define BMI160_ACCEL_DLPF_SEL_LEN   3

#define BMI160_ACCEL_RANGE_SEL_BIT  0
#define BMI160_ACCEL_RANGE_SEL_LEN  4

#define BMI160_CMD_START_FOC        0x03
#define BMI160_CMD_ACC_MODE_NORMAL  0x11
#define BMI160_CMD_GYR_MODE_NORMAL  0x15
#define BMI160_CMD_FIFO_FLUSH       0xB0
#define BMI160_CMD_INT_RESET        0xB1
#define BMI160_CMD_STEP_CNT_CLR     0xB2
#define BMI160_CMD_SOFT_RESET       0xB6

#define BMI160_RA_CMD               0x7E

//Delays, in milliseconds
#define BMI160_AUX_COM_DELAY            UINT8_C(10)
#define BMI160_READ_WRITE_DELAY         UINT8_C(1)

//Bit Masks for MAG_IF[1]
#define BMI160_MANUAL_MODE_EN_MSK       UINT8_C(0x80)
#define BMI160_AUX_READ_BURST_MSK       UINT8_C(0x03)

//** Enable/disable bit value */
#define BMI160_ENABLE                        UINT8_C(0x01)
#define BMI160_DISABLE                       UINT8_C(0x00)


/** Bandwidth settings */
/* Accel Bandwidth */
#define BMI160_ACCEL_BW_OSR4_AVG1            UINT8_C(0x00)
#define BMI160_ACCEL_BW_OSR2_AVG2            UINT8_C(0x01)
#define BMI160_ACCEL_BW_NORMAL_AVG4          UINT8_C(0x02)
#define BMI160_ACCEL_BW_RES_AVG8             UINT8_C(0x03)
#define BMI160_ACCEL_BW_RES_AVG16            UINT8_C(0x04)
#define BMI160_ACCEL_BW_RES_AVG32            UINT8_C(0x05)
#define BMI160_ACCEL_BW_RES_AVG64            UINT8_C(0x06)
#define BMI160_ACCEL_BW_RES_AVG128           UINT8_C(0x07)

#define BMI160_GYRO_BW_OSR4_MODE             UINT8_C(0x00)
#define BMI160_GYRO_BW_OSR2_MODE             UINT8_C(0x01)
#define BMI160_GYRO_BW_NORMAL_MODE           UINT8_C(0x02)

/**
    Gyroscope Sensitivity Range options
    @see setFullScaleGyroRange()
*/
typedef enum {
    BMI160_GYRO_RANGE_2000 = 0, /**<  +/- 2000 degrees/second */
    BMI160_GYRO_RANGE_1000,     /**<  +/- 1000 degrees/second */
    BMI160_GYRO_RANGE_500,      /**<  +/-  500 degrees/second */
    BMI160_GYRO_RANGE_250,      /**<  +/-  250 degrees/second */
    BMI160_GYRO_RANGE_125,      /**<  +/-  125 degrees/second */
} BMI160GyroRange;

/**
    Accelerometer Sensitivity Range options
    @see setFullScaleAccelRange()
*/
typedef enum {
    BMI160_ACCEL_RANGE_2G  = 0X03, /**<  +/-  2g range */
    BMI160_ACCEL_RANGE_4G  = 0X05, /**<  +/-  4g range */
    BMI160_ACCEL_RANGE_8G  = 0X08, /**<  +/-  8g range */
    BMI160_ACCEL_RANGE_16G = 0X0C, /**<  +/- 16g range */
} BMI160AccelRange;


// Initialization functions
uint8_t bmi160_init(void);
uint8_t bmi160_check_connection(void);

// Utility functions
void bmi160_set_gyro_range(uint16_t range) ;
void bmi160_set_full_scale_gyro_range(uint8_t range) ;
void bmi160_set_full_scale_accel_range(uint8_t range);
void bmi160_read_gyroscope(int16_t *x, int16_t *y, int16_t *z) ;
void bmi160_get_rotation(int16_t *x, int16_t *y, int16_t *z);
void bmi160_read_accelerometer(int16_t *x, int16_t *y, int16_t *z);
void bmi160_get_acceleration(int16_t *x, int16_t *y, int16_t *z);

// Helper functions
uint8_t reg_read (uint8_t reg);
void reg_write(uint8_t reg, uint8_t data);
void reg_write_bits(uint8_t reg, uint8_t data, uint8_t pos, uint8_t len) ;
uint8_t reg_read_bits(uint8_t reg, uint8_t pos, uint8_t len) ;
uint8_t read8(uint8_t reg);
void write8 (uint8_t reg, uint8_t value);
uint8_t spixfer(uint8_t x);