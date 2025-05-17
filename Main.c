/*
 * File:   Main.c
 * Author: krish
 *
 * Created on April 23, 2025, 9:22 PM
 */


#ifndef F_CPU
#define F_CPU 16000000UL // Define CPU frequency for delay functions
#endif

#include "Sensor_LCD.h" // Include LCD and ultrasonic function declarations
#define THRESHOLD_FREQUENCY 6200 // Frequency above which the buzzer is triggered 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>

// Global variables for frequency measurement
uint32_t value[5];       // Array to store captured timer values
uint8_t count = 0;       // Index to track number of captures
uint32_t real_freq[5];   // Array to store calculated frequencies
uint32_t nums = 0;       // Counter used for LCD update timing
uint16_t distance;


int main(void) 
{
    // Setup internal clock to 16MHz
    CCP = 0xd8;
    CLKCTRL.OSCHFCTRLA = 0x1C;
    while (CLKCTRL.MCLKSTATUS & 0b00000001); // Wait until clock is stable
    
    
    // Initialize LCD over I2C
    TWI0.MBAUD = 67;     // Set I2C baud rate
    TWI0.MCTRLA |= 0x01; // Enable TWI main.
    TWI0.MSTATUS = 0x01; // Force bus state machine to idle.

    // LCD Initialization
    LCDSetUp();

    
    // Metal Detector Initialization - Setup event system to route PC0 
    // signal to TCB2 input capture
    EVSYS.CHANNEL2 = 0x40;           // Event generator: PORTC pin 0
    EVSYS.USERTCB2CAPT = 0x03;       // TCB2 uses event channel 2

    // Configure TCB2 for frequency capture
    TCB2.EVCTRL = 0x01;           // Enable capture on event input
    TCB2.CTRLB = 0x03;            // Set TCB2 to frequency mode
    TCB2.CTRLA = 0x01;            // Enable TCB2 with no prescaling
    TCB2.INTCTRL = 0x01;          // Enable TCB2 interrupt



    sei(); // Enable global interrupts
   
    
    // Configure the AVR IO Pins
    PORTD.DIRSET = 0b00011110;  // Motor output from PCB is PD1-PD4
    PORTA.DIRSET = 0b00000010;  // TRIGGER pin of ultrasonic sensor is PA1
    PORTA.DIRCLR = 0b11110001;  // ECHO pin of ultrasonic sensor is PA0
                                // and RF receiver is PA7-PA4
    PORTC.DIRSET = 0b00000010;  // Buzzer is PC1
    
    // LCD + MOTORS Code
    while (1) 
    {
        // Update LCD every 200,000 loop iterations
        nums++;
        if(nums > 200000)
        {
            nums = 0;
            distance = get_distance();
            // Clear LCD screen before writing new distance
            LCDWrite(0b0000, 0, 0, 1);
            LCDWrite(0b0001, 0, 0, 1);
            _delay_ms(2);
            // Display the new distance on LCD
            LCDPrintNumber(distance);
            _delay_ms(2);
        }
        
        // Check RF buttons (connected to PA7?PA4) and control motors
        if(PORTA.IN & 0b10000000 || PORTA.IN & 0b01000000 || PORTA.IN & 0b00100000 || PORTA.IN & 0b00010000)
        {
            if(PORTA.IN & 0b10000000)
            {
                PORTD.OUT = 0b00001010; // Left backward, right side forward, 3RD BUTTON
            }

            if(PORTA.IN & 0b01000000)
            {
                PORTD.OUT = 0b00010100; // Left forward, right is backward, 4TH BUTTON
            }
            if(PORTA.IN & 0b00100000)
            {
                PORTD.OUT = 0b00010010;  // ALL FORWARD - 2ND BUTTON
            }
            if(PORTA.IN & 0b00010000)
            {
                PORTD.OUT = 0b00001100;  // ALL BACKWARD - 1ST BUTTON
            }
        }
        else
        {
                PORTD.OUT = 0b00000000; // Stop all motors
        }
    }
}

// ISR: captures frequency every time a rising edge is detected 
ISR(TCB2_INT_vect)
{
    TCB2.INTFLAGS = 0x01;             // Clear interrupt flag
    value[count] = TCB2.CCMP;         // Store captured timer count
    real_freq[count] = 16000000 / value[count];  // Calculate frequency
    
    // Activate buzzer if frequency exceeds threshold
    if(real_freq[count] > THRESHOLD_FREQUENCY)  
    {
        PORTC.OUT = 0b00000010;
    }
    else
    {
        PORTC.OUT = 0b00000000;
    }
   
    count++;
   
    if (count >= 5)
    {
        count = 0; // Wrap around after 5 readings
    }    
}
