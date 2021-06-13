/*
    spi.c

    Created: 16/12/2020 4:26:53 PM
    Author: user
*/

#include <spi.h>
#include <ioport.h>
#include <delay.h>

/*
    This function initializes and set up SPI
*/
void spi_init(void) {
    // Set Slave Select (SS) pin to High
    // De-select pin by default
    // NOTE: SS is pin 10 by default in hardware SPI now
    ioport_set_pin_high(SPI_HARDWARE_SS);

    // Set SS as output pin
    ioport_set_pin_dir(SPI_HARDWARE_SS, IOPORT_DIR_OUTPUT);

    // Initialize as SPI master
    SPCR |= (1 << MSTR);
    SPCR |= (1 << SPE);

    // Set SCK and MOSI as output pins
    // By doing this AFTER enabling SPI, we avoid accidentally
    // clocking in a single bit since the lines go directly
    // from "input" to SPI control.
    ioport_set_pin_dir(SPI_HARDWARE_SCK, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(SPI_HARDWARE_MOSI, IOPORT_DIR_OUTPUT);

    // MISO pin is override to Input automatically
    //ioport_set_pin_dir(SPI_HARDWARE_MISO, IOPORT_DIR_INPUT);
}