#include <asf.h>
#include <uart.h>

int main (void) {
    board_init();
    uart_init();

    while (1) {
        printf("Hello\r\n");
        delay_ms(20000);
    }
}
