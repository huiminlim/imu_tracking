#include <asf.h>
#include <uart.h>
#include <spi.h>
#include <bmi160.h>

int main (void) {
    board_init();
    uart_init();

    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;

    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;

    // Initialize SPI and BMI160 sensor
    spi_init();
    bmi160_init();

    if (bmi160_check_connection()) {
        printf("BMI160 connected\r\n\r\n");
    }
    else {
        printf("BMI160 connection failed\r\n\r\n");

        while (1) {
            delay_ms(20000);
        }
    }

    // Set scale for gyroscope and accelerometer
    bmi160_set_full_scale_gyro_range(BMI160_GYRO_RANGE_250);
    bmi160_set_full_scale_accel_range(BMI160_ACCEL_RANGE_2G);
    printf("Gyroscope range: %d deg/s\r\n", bmi160_get_full_scale_gyro_range());
    printf("Accelerometer range: %d g\r\n", bmi160_get_full_scale_accel_range());
    printf("\r\n");

    // TODO: Offsets - how to handle?

    while (1) {
        bmi160_read_accelerometer(&ax_raw, &ay_raw, &az_raw);
        printf("(raw) ");
        printf("ax: %d ", ax_raw);
        printf("ay: %d ", ay_raw);
        printf("az: %d\r\n", az_raw);

        bmi160_read_gyroscope(&gx_raw, &gy_raw, &gz_raw);
        printf("(raw) ");
        printf("gx: %d ", gx_raw);
        printf("gy: %d ", gy_raw);
        printf("gz: %d\r\n", gz_raw);

        printf("\r\n");
        delay_ms(1000);
    }
}
