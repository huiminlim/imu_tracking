#include <asf.h>
#include <uart.h>
#include <spi.h>
#include <bmi160.h>

int main (void) {
    board_init();
    uart_init();

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t ax;
    int16_t ay;
    int16_t az;

    // Initialize SPI and BMI160 sensor
    spi_init();
    int ret = bmi160_init();

    if (ret == BMI160_CHIP_ID) {
        uint8_t sensorID = reg_read(BMI160_RA_CHIP_ID);
        printf("BMI160 id: 0x%x\r\n", sensorID);
    }
    else {
        printf("BMI160 init failed\r\n");
    }

    while (1) {
        read_accelerometer(&ax, &ay, &az);
        printf("ax: %d ", ax);
        printf("ay: %d ", ay);
        printf("az: %d\r\n", az);

        read_gyro(&gx, &gy, &gz);
        printf("gx: %d ", gx);
        printf("gy: %d ", gy);
        printf("gz: %d\r\n", gz);

        printf("\r\n");
        delay_ms(20000);
    }
}
