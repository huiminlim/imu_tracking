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
