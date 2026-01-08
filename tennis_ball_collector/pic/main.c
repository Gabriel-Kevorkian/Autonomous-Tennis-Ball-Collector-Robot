#include <xc.h>
#include <pic18f4550.h>
#include <stdint.h>
#include <stdbool.h>
#include "conbits.h"

#define PW1 PORTDbits.RD4
#define PW2 PORTDbits.RD5
#define PW3 PORTDbits.RD6
#define PW4 PORTDbits.RD7

#define D1 PORTBbits.RB0
#define D2 PORTBbits.RB1
#define D3 PORTBbits.RB2
#define D4 PORTBbits.RB3
#define C0 PORTCbits.RC0
#define C1 PORTCbits.RC1

#define LED PORTDbits.RD1

void init(void);
void T0Delay(void);
void blinkLED(void);
void stopMotor(void);
void move(unsigned char direction, unsigned int distance);
void turn(unsigned char direction, unsigned int angle);
void move_rover(unsigned char letter);
void catch_ball(void);

// === GLOBAL VARIABLE ===
unsigned char received_char = 0;

// === INTERRUPT SERVICE ROUTINE ===
void __interrupt(high_priority) high_isr(void) {
    // UART Receive Interrupt
    if (PIR1bits.RCIF) {
        if (RCSTAbits.OERR) {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }
        received_char = RCREG;  // Read the received byte
        PIR1bits.RCIF = 0;      // Clear flag
        move_rover(received_char);
    }
}

// === MAIN PROGRAM ===
void main(void) {
    init();
    
    while (1) {
        // Main loop - all work done in interrupts
    }
}

void init(){
    OSCCONbits.IRCF = 0x07;
    OSCCONbits.SCS = 0x03;
    while(OSCCONbits.IOFS != 1); // 8 MHz
    
    ADCON1 = 0x0F;
    
    // Motor control pins (outputs)
    TRISDbits.TRISD1 = 0; // LED
    TRISDbits.TRISD4 = 0; // PW1
    TRISDbits.TRISD5 = 0; // PW2
    TRISDbits.TRISD6 = 0; // PW3
    TRISDbits.TRISD7 = 0; // PW4
    
    TRISBbits.TRISB0 = 0; // D1
    TRISBbits.TRISB1 = 0; // D2
    TRISBbits.TRISB2 = 0; // D3
    TRISBbits.TRISB3 = 0; // D4
    
    TRISCbits.TRISC0 = 0; // D3
    TRISCbits.TRISC1 = 0;
    // Initialize all outputs to 0
    PW1 = 0;
    PW2 = 0;
    PW3 = 0;
    PW4 = 0;
    LED = 0;
    D1 = 0;
    D2 = 0;
    D3 = 0;
    D4 = 0;
    C0 = 0;
    C1 = 0;

    // UART Pins
    TRISCbits.RC6 = 0; // TX output
    TRISCbits.RC7 = 1; // RX input

    // UART Config for 9600 baud @ 8MHz
    TXSTAbits.SYNC = 0;     // Asynchronous
    TXSTAbits.BRGH = 1;     // High speed
    BAUDCONbits.BRG16 = 1;  // 16-bit baud rate
    SPBRG = 207;            // 9600 baud @ 8MHz
    SPBRGH = 0;

    RCSTAbits.SPEN = 1;     // Enable Serial Port
    RCSTAbits.CREN = 1;     // Enable continuous receive
    TXSTAbits.TXEN = 1;     // Enable transmitter

    // Enable UART receive interrupt (high priority)
    PIE1bits.RCIE = 1;      // Enable UART RX interrupt
    IPR1bits.RCIP = 1;      // Set high priority
    RCONbits.IPEN = 1;      // Enable interrupt priority
    INTCONbits.GIEH = 1;    // Enable high-priority interrupts
    INTCONbits.GIEL = 1;    // Enable low-priority interrupts
}

// === MOVE ROVER BASED ON COMMAND ===
// Modified for ball tracking - shorter movements for better control
void move_rover(unsigned char letter) {
    if (letter == 'W') {
        // Move forward - shorter distance for tracking
        move('F', 1);  // About 10-15cm forward
    } 
    else if (letter == 'S') {
        // Move backward
        move('B', 1);
    } 
    else if (letter == 'A') {
        // Turn left - smaller angle for precise tracking
        turn('L', 1);  // About 20-25 degrees
    } 
    else if (letter == 'D') {
        // Turn right - smaller angle for precise tracking
        turn('R', 1);  // About 20-25 degrees
    } 
    else if (letter == 'C') {
        catch_ball();
    } 
    else if (letter == 'X') {
        // Emergency stop command
        stopMotor();
        blinkLED();
    }
    else {
        // Unknown command - blink LED
        blinkLED();
    }
}

void catch_ball(){
    C0=1;
    C1=0;
    unsigned int i;

    i = 1000;
    while(i != 0){
        T0Delay();
        i -= 1;
    }
    C0=0;
    C1=1;
    i = 1000;
    while(i != 0){
        T0Delay();
        i -= 1;
    }
    C0=0;
    C1=0;
    
}


void move(unsigned char direction, unsigned int distance){
    unsigned int y;

    while(distance != 0){
        if(direction == 'F'){
            // Move forward
            y = 13;  // Reduced from 465 for shorter movements
            D1 = 1;
            D3 = 1;
            D2 = 0;
            D4 = 0;
            PW1 = 1;
            PW3 = 1;
        } 
        else if(direction == 'B'){
            // Move backward
            y = 30;  // Reduced from 500
            D1 = 0;
            D3 = 0;
            D2 = 1;
            D4 = 1;
            PW2 = 1;
            PW4 = 1;
        }
        
        // Run motors for specified time
        while(y != 0){
            T0Delay();
            y -= 1;
        }
        
        stopMotor();
        

        
        distance -= 1;
    }
    
    stopMotor();
    blinkLED();
}

void turn(unsigned char direction, unsigned int angle){
    unsigned char x;
    
    while(angle != 0){
        if(direction == 'L'){
            // Turn left
            x = 10;  // Reduced from 210 for smaller turns
            D1 = 0;
            D3 = 1;
            D2 = 1;
            D4 = 0;
            PW2 = 1;
            PW3 = 1;
        } 
        else if(direction == 'R'){
            // Turn right
            x = 10;  // Reduced from 215 for smaller turns
            D1 = 1;
            D3 = 0;
            D2 = 0;
            D4 = 1;
            PW1 = 1;
            PW4 = 1;   
        }
        
        // Run motors for turn duration
        while(x != 0){
            T0Delay();
            x -= 1;
        }
        
        stopMotor();
        
        // Short pause after turn
        unsigned int i = 100;   
        while(i != 0){
            T0Delay();
            i -= 1;
        }
        
        angle -= 1;
    }
    
    stopMotor();
    blinkLED();
}

void stopMotor(){
    PW1 = 0;
    PW2 = 0;
    PW3 = 0;
    PW4 = 0;
}

void blinkLED(){
    unsigned char i;
    LED = 1;
    i = 50;
    while(i != 0){
        T0Delay();
        i -= 1;
    }
    LED = 0;
}

// 1 ms delay using Timer0
void T0Delay(){
    T0CON = 0x08;
    TMR0H = 0xF8;
    TMR0L = 0x30;
    T0CONbits.TMR0ON = 1;
    while(INTCONbits.TMR0IF == 0);
    T0CONbits.TMR0ON = 0;
    INTCONbits.TMR0IF = 0;
}