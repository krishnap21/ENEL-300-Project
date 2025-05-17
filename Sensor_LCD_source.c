/*
 * File:   sensor lcd code.c
 * Author: krish
 *
 * Created on April 23, 2025, 9:23 PM
 */

#include "Sensor_LCD.h"


// Low-level I2C write to send a byte to a given address
void I2CWrite(uint8_t address, uint8_t data) 
{
    TWI0.MADDR = address << 1;
    while (!(TWI0.MSTATUS & 0b01000000));
    TWI0.MDATA = data;
    while (!(TWI0.MSTATUS & 0b01000000));
    TWI0.MCTRLB = 0x03;
    _delay_ms(1);
}

// LCD Write Function
void LCDWrite(uint8_t nibble, uint8_t rs, uint8_t rw, uint8_t ledOn) {
    uint8_t data = (nibble << 4) | (ledOn << 3) | (rw << 1) | rs;
    I2CWrite(DISPLAY_ADDR, data);
    data |= 0b00000100;
    I2CWrite(DISPLAY_ADDR, data);  
    data &= 0b11111011;
    I2CWrite(DISPLAY_ADDR, data);
}

// Function to Print a Number to LCD
void LCDPrintNumber(uint16_t num) {
    char buffer[6];  
    uint8_t i = 0;
    do {
        buffer[i++] = (num % 10) + '0';  
        num /= 10;
    } while (num > 0);
    
    buffer[i] = '\0';  

    for (int j = i - 1; j >= 0; j--) {
        LCDWrite(buffer[j] >> 4, 1, 0, 1);  
        LCDWrite(buffer[j] & 0x0F, 1, 0, 1);  
    }
}

// Measure distance using an ultrasonic sensor
uint16_t get_distance() {
    uint16_t pulse_width = 0;

    // Ensure TRIG is LOW
    PORTA.OUTCLR = TRIG_PIN;
    _delay_us(5);

    // Send a 10µs HIGH pulse to trigger the sensor
    PORTA.OUTSET = TRIG_PIN;
    _delay_us(10);
    PORTA.OUTCLR = TRIG_PIN;

    // Wait for ECHO to go HIGH (timeout if stuck)
    uint16_t timeout = 0;
    while (!(PORTA.IN & ECHO_PIN)) {
        if (++timeout > 30000) return 9999;  // Return error if no signal
    }

    TCA0.SINGLE.CTRLA = 0;  // Stop Timer
    TCA0.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESET_gc; // Reset Timer
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; // Start Timer (1MHz = 1us per tick)

    // Wait for ECHO to go LOW
    while (PORTA.IN & ECHO_PIN) {
        if (TCA0.SINGLE.CNT > 30000) return 9999;  // Timeout error
    }
    pulse_width = TCA0.SINGLE.CNT;  // Save the pulses duration

    // Convert time to distance in cm
    return (pulse_width * 0.0343) / 2;
}


void LCDSetUp()
{
    // Initialization steps required by LCD - given to us
    LCDWrite(0b0011, 0, 0, 1);
    _delay_ms(5);
    LCDWrite(0b0011, 0, 0, 1);
    _delay_ms(5);
    LCDWrite(0b0011, 0, 0, 1);
    _delay_ms(5);

    // Switch from 8-bit to 4-bit mode.
    LCDWrite(0b0010, 0, 0, 1);

    // Set to 4-bit operation with 1 display line. 
    LCDWrite(0b0010, 0, 0, 1);
    LCDWrite(0b0000, 0, 0, 1);

    // Display on, cursor on, cursor flash off.
    LCDWrite(0b0000, 0, 0, 1);
    LCDWrite(0b1110, 0, 0, 1);

    // Cursor increment when write on, display shift off.
    LCDWrite(0b0000, 0, 0, 1);
    LCDWrite(0b0110, 0, 0, 1);

    // Clear screen
    LCDWrite(0b0000, 0, 0, 1);
    LCDWrite(0b0001, 0, 0, 1);
}