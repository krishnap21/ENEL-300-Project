#ifndef ULTRASONIC_LCD_H
#define ULTRASONIC_LCD_H

#ifndef F_CPU
#define F_CPU 16000000UL // CPU frequency required for _delay functions
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define DISPLAY_ADDR 0x27 // I2C address of the LCD display
#define TRIG_PIN  0b00000010  // PA1
#define ECHO_PIN  0b00000001  // PA0

void I2CWrite(uint8_t address, uint8_t data); // Low-level I2C write
void LCDWrite(uint8_t nibble, uint8_t rs, uint8_t rw, uint8_t ledOn); // Send command/data to LCD
void LCDPrintNumber(uint16_t num); // Display number on LCD
void LCDSetUp();  // Setup LCD
uint16_t get_distance(void); // Get distance from ultrasonic sensor

#endif // ULTRASONIC_LCD_H
