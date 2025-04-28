////blinking code
//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//
//// --- Pin Definitions ---
//#define PIR_SENSOR_PIN PORTAbits.RA0    // PIR sensor input (RA0)
//#define LED_POWER LATBbits.LATB7        // Power LED (RB7)
//#define LED_TX LATBbits.LATB2           // TX LED / Activity LED (RB2)
//
//// --- Function Prototypes ---
//void initLEDs(void);
//void delay_short(void);
//
//int main(void) {
//    // 1. Disable all analog inputs (make all pins digital)
//    AD1PCFGL = 0xFFFF;
//
//    // 2. Set I/O directions
//    TRISAbits.TRISA0 = 1;   // PIR input (unused for now)
//    TRISBbits.TRISB2 = 0;   // TX/Activity LED output
//    TRISBbits.TRISB7 = 0;   // Power LED output
//
//    // 3. Initialize outputs
//    initLEDs();
//
//    // 4. Main loop: blink both LEDs in an alternating pattern
//    while (1) {
//        LED_POWER = 1;
//        LED_TX = 0;
//        delay_short();
//
//        LED_POWER = 0;
//        LED_TX = 1;
//        delay_short();
//    }
//
//    return 0;
//}
//
//// --- Initialize LED states ---
//void initLEDs(void) {
//    LED_POWER = 1;   // Power LED initially ON
//    LED_TX = 0;      // TX/Activity LED initially OFF
//}
//
//// --- Simple blocking delay ---
//void delay_short(void) {
//    for (volatile unsigned int i = 0; i < 50000; i++);
//}






//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"  // Your configuration bits should be in here
//
//// Define UART pins
//#define UART_TX_PIN LATBbits.LATB3 // UART TX (RB3)
//#define UART_RX_PIN LATBbits.LATB2 // UART RX (RB2)
//
//// Define sensor pins
//#define PIR_SENSOR_PIN PORTAbits.RA0    // PIR Sensor (RA0)
//#define SCL_PIN LATBbits.LATB8         // I2C SCL (RB8)
//#define SDA_PIN LATBbits.LATB9         // I2C SDA (RB9)
//
//// Define LED pins
//#define LED_POWER LATBbits.LATB7  // Power LED (Always ON)
//#define LED_TX LATBbits.LATB2     // LED blinks when sending UART messages
//
//// Function prototypes
//void initUART(void);
//void sendUARTMessage(const char *message);
//void initI2C(void);
//void initLEDs(void);
//void blinkLED2(void);
//
//int main() {
//    // 1. Disable analog on all pins (use as digital)
//    AD1PCFGL = 0xFFFF; // All ANx pins set to digital mode
//
//    // 2. Set pin directions
//    TRISAbits.TRISA0 = 1;  // PIR sensor input
//    TRISBbits.TRISB7 = 0;  // Power LED output
//    TRISBbits.TRISB2 = 0;  // TX LED output
//
//
//    LED_POWER = 1; // Power LED always ON
//    LED_TX = 0;    // Transmission LED initially OFF
//
//    // 3. Configure UART pins via PPS (Peripheral Pin Select)
//    __builtin_write_OSCCONL(OSCCON & 0xBF);  // Unlock PPS
//
//    RPINR18bits.U1RXR = 2; // UART1 RX on RP2 (RB2)
//    RPOR1bits.RP3R = 3;    // UART1 TX on RP3 (RB3)
//
//    __builtin_write_OSCCONL(OSCCON | 0x40);  // Lock PPS
//
//    // 4. Initialize peripherals
//    //initUART();
//    //sendUARTMessage("UART initialized");
//    //sendUARTMessage("Hello, world!");  // This should appear in MPLAB Data Visualizer
//
//    //initI2C();
//    //sendUARTMessage("I2C initialized");
//
//    initLEDs();
//    //sendUARTMessage("POWERLED ON");
//
//    while (1) {
//        unsigned int i;
//      //  if (PIR_SENSOR_PIN == 1) {
//      //      sendUARTMessage("motion triggered");
//      //      sendUARTMessage("trigger system");
//      //  }
//
//    LED_POWER = 1; // Power LED always ON
//    LED_TX = 0;    // Transmission LED initially OFF
//        for (i = 0; i < 50000; i++); // Simple delay loop
//    LED_POWER = 0; // Power LED always ON
//    LED_TX = 1;    // Transmission LED initially OFF
//        for (i = 0; i < 50000; i++); // Simple delay loop
//    }
//
//    return 0;
//}
//
//// UART Initialization
//void initUART(void) {
//    U1MODE = 0x8000;  // Enable UART, 8-bit, no parity, 1 stop bit
//    U1BRG = 8;        // Baud rate for 115200 bps (Fcy = 16 MHz)
//    U1STA = 0x0400;   // Enable TX
//}
//
//// Send a message over UART and blink LED2
//void sendUARTMessage(const char *message) {
//    blinkLED2(); // Indicate transmission activity
//    while (*message) {
//        while (U1STAbits.UTXBF); // Wait if TX buffer is full
//        U1TXREG = *message++;    // Send character
//    }
//    while (!U1STAbits.TRMT); // Wait for last char to transmit
//    U1TXREG = '\n';          // Send newline
//}
//
//// I2C Initialization
//void initI2C(void) {
//    I2C1BRG = 25;            // Set baud rate for 100 kHz
//    I2C1CONbits.I2CEN = 1;   // Enable I2C module
//}
//
//// Initialize LEDs
//void initLEDs(void) {
//    LED_POWER = 1; // Power LED always ON
//    LED_TX = 0;    // Transmission LED initially OFF
//}
//
//// Blink LED2 when sending data
//void blinkLED2(void) {
//    LED_TX = 1;
//    for (volatile int i = 0; i < 10000; i++); // Short delay
//    LED_TX = 0;
//}




//
//
//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//
//// Pin definitions
//#define PIR_SENSOR_PIN PORTAbits.RA0   // PIR sensor (RA0)
//#define LED_POWER LATBbits.LATB7       // Power LED (RB7)
//#define LED_ACTIVITY LATBbits.LATB2    // LED 1 (RB2)
//
//// Function prototypes
//void initUART(void);
//void sendUARTMessage(const char *message);
//void initLEDs(void);
//void blinkLEDActivityOnce(void);
//void delay_ms(unsigned int ms);
//
//int main() {
//    AD1PCFGL = 0xFFFF;
//
//    TRISAbits.TRISA0 = 1;  // PIR input
//    TRISBbits.TRISB2 = 0;  // Activity LED
//    TRISBbits.TRISB7 = 0;  // Power LED
//
//    __builtin_write_OSCCONL(OSCCON & 0xBF);
//    RPINR18bits.U1RXR = 14;
//    RPOR5bits.RP11R = 3;
//    __builtin_write_OSCCONL(OSCCON | 0x40);
//
//    initUART();
//    initLEDs();
//    sendUARTMessage("System initialized");
//
//    while (1) {
//        if (PIR_SENSOR_PIN == 1) {
//            sendUARTMessage("PIR was activated");
//            blinkLEDActivityOnce();
//
//            // Cooldown: ignore any further triggers for 0 seconds
//            delay_ms(500);
//        }
//    }
//
//
//
//    return 0;
//}
//
//void initUART(void) {
//    U1MODE = 0x8000;
//    U1BRG = 8;
//    U1STA = 0x0400;
//}
//
//void sendUARTMessage(const char *message) {
//    while (*message) {
//        while (U1STAbits.UTXBF);
//        U1TXREG = *message++;
//    }
//    while (!U1STAbits.TRMT);
//    U1TXREG = '\n';
//}
//
//void initLEDs(void) {
//    LED_POWER = 1;
//    LED_ACTIVITY = 0;
//}
//
//void blinkLEDActivityOnce(void) {
//    LED_ACTIVITY = 1;
//    delay_ms(2000);  // LED on for 2 seconds
//    LED_ACTIVITY = 0;
//}
//
//void delay_ms(unsigned int ms) {
//    while (ms--) {
//        for (volatile unsigned int i = 0; i < 1600; i++);
//    }
//}
//
//





//-----veml7700 pir code 

//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//
//// --- I2C Definitions ---
//#define I2C_BAUD 157  // For ~100kHz at 16MHz Fcy
//#define VEML7700_ADDR 0x10 << 1  // VEML7700 I2C address (left-shifted)
//
// //--- Pin definitions ---
//#define PIR_SENSOR_PIN PORTAbits.RA0   // PIR sensor (RA0)
//#define LED_POWER LATBbits.LATB7       // Power LED (RB7)
//#define LED_ACTIVITY LATBbits.LATB2    // Activity LED (RB2)
//
//// --- Function prototypes ---
//void initUART(void);
//void sendUARTMessage(const char *message);
//void initLEDs(void);
//void blinkLEDActivityOnce(void);
//void delay_ms(unsigned int ms);
//
//void initI2C(void);
//void I2CStart(void);
//void I2CStop(void);
//void I2CSend(uint8_t data);
//uint8_t I2CRead(bool ack);
//void VEML7700_Init(void);
//uint16_t VEML7700_ReadLuxRaw(void);
//float calculateLux(uint16_t raw);
//
//int main() {
//    AD1PCFGL = 0xFFFF;
//
//    TRISAbits.TRISA0 = 1;    // PIR input
//    TRISBbits.TRISB2 = 0;    // Activity LED (LED1)
//    TRISBbits.TRISB7 = 0;    // Power LED
//
//    // PPS (Peripheral Pin Select) for UART
//    __builtin_write_OSCCONL(OSCCON & 0xBF);  // Unlock PPS
//    RPINR18bits.U1RXR = 14;                  // UART RX on RP14 (RB14)
//    RPOR5bits.RP11R = 3;                     // UART TX on RP11 (RB11)
//    __builtin_write_OSCCONL(OSCCON | 0x40);  // Lock PPS
//
//    initUART();
//    initLEDs();
//    initI2C();
//    VEML7700_Init();
//
//    sendUARTMessage("System initialized");
//
//    while (1) {
//        float lux = calculateLux(VEML7700_ReadLuxRaw());
//
//        char msg[64];
//        sprintf(msg, "Lux = %.2f lx", lux);
//        sendUARTMessage(msg);
//
//        if (lux < 20.0) {
//            sendUARTMessage("Low light detected! Triggering LED1.");
//            blinkLEDActivityOnce();
//        }
//
//        if (PIR_SENSOR_PIN == 1) {
//            sendUARTMessage("PIR was activated");
//            blinkLEDActivityOnce();
//            delay_ms(500);  // Cooldown
//        }
//
//        delay_ms(1000);  // Polling interval
//    }
//
//    return 0;
//}
//
//void initUART(void) {
//    U1MODE = 0x8000;  // Enable UART
//    U1BRG = 8;        // 115200 baud at 16MHz Fcy
//    U1STA = 0x0400;   // Enable TX
//}
//
//void sendUARTMessage(const char *message) {
//    while (*message) {
//        while (U1STAbits.UTXBF);
//        U1TXREG = *message++;
//    }
//    while (!U1STAbits.TRMT);
//    U1TXREG = '\n';
//}
//
//void initLEDs(void) {
//    LED_POWER = 1;
//    LED_ACTIVITY = 0;
//}
//
//void blinkLEDActivityOnce(void) {
//    LED_ACTIVITY = 1;
//    delay_ms(2000);  // On for 2 seconds
//    LED_ACTIVITY = 0;
//}
//
//void delay_ms(unsigned int ms) {
//    while (ms--) {
//        for (volatile unsigned int i = 0; i < 1600; i++);
//    }
//}
//
//// --- I2C and VEML7700 Functions ---
//
//void initI2C(void) {
//    I2C1BRG = I2C_BAUD;
//    I2C1CONbits.I2CEN = 1;
//}
//
//void I2CStart(void) {
//    I2C1CONbits.SEN = 1;
//    while (I2C1CONbits.SEN);
//}
//
//void I2CStop(void) {
//    I2C1CONbits.PEN = 1;
//    while (I2C1CONbits.PEN);
//}
//
//void I2CSend(uint8_t data) {
//    I2C1TRN = data;
//    while (I2C1STATbits.TRSTAT);       // Wait for transmission
//    while (I2C1STATbits.ACKSTAT);      // Wait for ACK
//}
//
//uint8_t I2CRead(bool ack) {
//    I2C1CONbits.RCEN = 1;              // Enable receive mode
//    while (!I2C1STATbits.RBF);
//    uint8_t data = I2C1RCV;
//    I2C1CONbits.ACKDT = !ack;          // ACK = 1 to continue, 0 to stop
//    I2C1CONbits.ACKEN = 1;             // Send ACK
//    while (I2C1CONbits.ACKEN);
//    return data;
//}
//
//void VEML7700_Init(void) {
//    I2CStart();
//    I2CSend(VEML7700_ADDR | 0);  // Write mode
//    I2CSend(0x00);               // ALS_CONFIG register
//    I2CSend(0x00);               // Low byte (gain, integration time)
//    I2CSend(0x00);               // High byte (default settings)
//    I2CStop();
//}
//
//uint16_t VEML7700_ReadLuxRaw(void) {
//    I2CStart();
//    I2CSend(VEML7700_ADDR | 0);
//    I2CSend(0x04);  // ALS_DATA register
//    I2CStop();
//
//    I2CStart();
//    I2CSend(VEML7700_ADDR | 1);
//    uint8_t lsb = I2CRead(true);
//    uint8_t msb = I2CRead(false);
//    I2CStop();
//
//    return (msb << 8) | lsb;
//}
//
//float calculateLux(uint16_t raw) {
//    return raw * 0.0576f;  // Approximate for default gain/integration
//}


////pir only
//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//
//// --- Timing Constants ---
//#define LIGHT_ON_TIME_MS 20000UL     // 20 seconds
//#define COOLDOWN_MS 120000UL         // 2 minutes
//#define LOOP_DELAY_MS 100UL          // 100ms resolution
//
//// --- Pin Definitions (do not modify)
//#define PIR1_SENSOR_PIN PORTAbits.RA0     // PIR 1 input
//#define LED_POWER LATBbits.LATB7          // Power LED (always on)
//#define LED_ACTIVITY LATBbits.LATB2       // Activity LED (triggered by PIR)
//#define RS485_DE_RE_LAT LATBbits.LATB4    // DE/RE pin
//#define RS485_DE_RE_TRIS TRISBbits.TRISB4 // TRIS for DE/RE pin
//
//// --- Function Prototypes ---
//void initUART(void);
//void sendUARTMessage(const char *message);
//void initLEDs(void);
//void delay_ms(unsigned long ms);
//void initRS485(void);
//
//int main(void) {
//    AD1PCFGL = 0xFFFF;  // All analog pins digital
//
//    // Set directions
//    TRISAbits.TRISA0 = 1;   // PIR1 input
//    TRISBbits.TRISB2 = 0;   // LED_ACTIVITY output
//    TRISBbits.TRISB7 = 0;   // LED_POWER output
//    RS485_DE_RE_TRIS = 0;   // RS-485 DE/RE pin
//
//    // Peripheral Pin Select (PPS)
//    __builtin_write_OSCCONL(OSCCON & 0xBF);  // Unlock PPS
//    RPINR18bits.U1RXR = 14;                  // RX on RP14 (RB14)
//    RPOR5bits.RP11R = 3;                     // TX on RP11 (RB3)
//    __builtin_write_OSCCONL(OSCCON | 0x40);  // Lock PPS
//
//    initUART();
//    initLEDs();
//    initRS485();
//
//    sendUARTMessage("System initialized");
//
//    bool inCooldown = false;
//    unsigned long cooldownTimer = 0;
//
//    bool lightActive = false;
//    unsigned long lightTimer = 0;
//
//    while (1) {
//        // PIR Trigger
//        if (!inCooldown && PIR1_SENSOR_PIN == 1) {
//            sendUARTMessage("PIR 1 was activated");
//
//            LED_ACTIVITY = 1;
//            lightActive = true;
//            lightTimer = LIGHT_ON_TIME_MS;
//
//            inCooldown = true;
//            cooldownTimer = COOLDOWN_MS;
//        }
//
//        // Handle Activity LED
//        if (lightActive) {
//            if (lightTimer > 0) {
//                lightTimer -= LOOP_DELAY_MS;
//            } else {
//                LED_ACTIVITY = 0;
//                lightActive = false;
//            }
//        }
//
//        // Handle Cooldown
//        if (inCooldown) {
//            if (cooldownTimer > 0) {
//                cooldownTimer -= LOOP_DELAY_MS;
//            } else {
//                inCooldown = false;
//            }
//        }
//
//        delay_ms(LOOP_DELAY_MS);  // Single delay for timing accuracy
//    }
//
//    return 0;
//}
//
//void initUART(void) {
//    U1MODE = 0x8000;  // Enable UART
//    U1BRG = 8;        // 115200 baud @ 16MHz Fcy
//    U1STA = 0x0400;   // Enable TX
//}
//
//void initRS485(void) {
//    RS485_DE_RE_LAT = 0;  // Start in receive mode
//}
//
//void sendUARTMessage(const char *message) {
//    RS485_DE_RE_LAT = 1;  // Enable transmit
//
//    while (*message) {
//        while (U1STAbits.UTXBF);  // Wait if TX buffer full
//        U1TXREG = *message++;
//    }
//
//    while (!U1STAbits.TRMT);  // Wait for last char to send
//    U1TXREG = '\n';
//    while (!U1STAbits.TRMT);
//    delay_ms(1);
//
//    RS485_DE_RE_LAT = 0;  // Return to receive
//}
//
//void initLEDs(void) {
//    LED_POWER = 1;       // Power LED always on
//    LED_ACTIVITY = 0;    // Activity LED off at start
//}
//
//// ? Calibrated delay (adjust inner loop for accuracy)
//void delay_ms(unsigned long ms) {
//    while (ms--) {
//        for (volatile unsigned int i = 0; i < 250; i++);  // ~1ms @ 16 MIPS
//    }
//}
//
//


//
//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//
//#define USE_VEML7700        // Comment this out if not using the ambient light sensor
//
//// --- I2C Definitions ---
//#define I2C_BAUD 157
//#define VEML7700_ADDR (0x10 << 1)
//#define DARK_LUX_THRESHOLD 10.0f
//
//// --- Cooldown Time ---
//#define COOLDOWN_MS 300000UL  // 5 minutes (UL = unsigned long)
//
//// --- Pin Definitions ---
//#define PIR1_SENSOR_PIN PORTAbits.RA0     // PIR 1 input
//#define LED_POWER LATBbits.LATB7          // Power LED
//#define LED_ACTIVITY LATBbits.LATB2       // Activity LED
//#define RS485_DE_RE_LAT LATBbits.LATB4    // DE/RE pin
//#define RS485_DE_RE_TRIS TRISBbits.TRISB4 // TRIS for DE/RE pin
//
//// --- Function Prototypes ---
//void initUART(void);
//void sendUARTMessage(const char *message);
//void initLEDs(void);
//void delay_ms(unsigned long ms);
//void blinkActivityLED(void);
//void initRS485(void);
//
//bool lightActive = false;
//unsigned long lightTimer = 0;
//
//
//#ifdef USE_VEML7700
//void initI2C(void);
//void I2CStart(void);
//void I2CStop(void);
//void I2CSend(uint8_t data);
//uint8_t I2CRead(bool ack);
//void VEML7700_Init(void);
//uint16_t VEML7700_ReadLuxRaw(void);
//float calculateLux(uint16_t raw);
//#endif
//
//
//int main(void) {
//    AD1PCFGL = 0xFFFF;
//
//    TRISAbits.TRISA0 = 1;   // PIR1 input
//    TRISBbits.TRISB2 = 0;   // Activity LED
//    TRISBbits.TRISB7 = 0;   // Power LED (LED 1)
//    RS485_DE_RE_TRIS = 0;
//
//    __builtin_write_OSCCONL(OSCCON & 0xBF);
//    RPINR18bits.U1RXR = 14;
//    RPOR5bits.RP11R = 3;
//    __builtin_write_OSCCONL(OSCCON | 0x40);
//
//    initUART();
//    initLEDs();
//    initRS485();
//
//#ifdef USE_VEML7700
//    initI2C();
//    VEML7700_Init();
//#endif
//
//    sendUARTMessage("System initialized");
//
//    bool inCooldown = false;
//    unsigned long cooldownTimer = 0;
//
//    bool lightActive = false;
//    unsigned long lightTimer = 0;
//
//#ifdef USE_VEML7700
//    float lux = 0;
//#endif
//
//    while (1) {
//#ifdef USE_VEML7700
//        uint16_t rawLux = VEML7700_ReadLuxRaw();
//        lux = calculateLux(rawLux);
//#endif
//
//        bool shouldTriggerLight = false;
//
//        // PIR triggered and it's dark
//        if (!inCooldown && PIR1_SENSOR_PIN == 1) {
//#ifdef USE_VEML7700
//            if (lux < DARK_LUX_THRESHOLD)
//#endif
//            {
//                sendUARTMessage("PIR 1 was activated");
//                blinkActivityLED();
//                shouldTriggerLight = true;
//                inCooldown = true;
//                cooldownTimer = 300000UL;
//            }
//        }
//
//#ifdef USE_VEML7700
//        // Also allow light-only trigger (ambient darkness)
//        if (lux < DARK_LUX_THRESHOLD) {
//            shouldTriggerLight = true;
//        }
//#endif
//
//        if (shouldTriggerLight && !lightActive) {
//            lightActive = true;
//            lightTimer = 20000UL;
//            LED_POWER = 1;
//        }
//
//        if (lightActive) {
//            if (lightTimer > 0) {
//                delay_ms(10);
//                lightTimer -= 10;
//            } else {
//                LED_POWER = 0;
//                lightActive = false;
//            }
//        }
//
//        if (inCooldown) {
//            if (cooldownTimer > 0) {
//                delay_ms(10);
//                cooldownTimer -= 10;
//            } else {
//                inCooldown = false;
//            }
//        }
//
//        delay_ms(10);
//    }
//
//    return 0;
//}
//
//
//
//void initUART(void) {
//    U1MODE = 0x8000;  // Enable UART
//    U1BRG = 8;        // 115200 baud @ 16MHz Fcy
//    U1STA = 0x0400;   // Enable TX
//}
//
//void initRS485(void) {
//    RS485_DE_RE_LAT = 0;  // Receive mode (default)
//}
//
//void sendUARTMessage(const char *message) {
//    RS485_DE_RE_LAT = 1;  // Enable transmit
//
//    while (*message) {
//        while (U1STAbits.UTXBF);  // Wait if TX buffer full
//        U1TXREG = *message++;
//    }
//
//    while (!U1STAbits.TRMT);  // Wait for all data
//    U1TXREG = '\n';           // Line ending
//    while (!U1STAbits.TRMT);
//    delay_ms(1);              // Allow line to settle
//
//    RS485_DE_RE_LAT = 0;      // Back to receive mode
//}
//
//void initLEDs(void) {
//    LED_POWER = 1;
//    LED_ACTIVITY = 0;
//}
//
//void delay_ms(unsigned long ms) {
//    while (ms--) {
//        for (volatile unsigned int i = 0; i < 1600; i++);
//    }
//}
//
//void blinkActivityLED(void) {
//    LED_ACTIVITY = 1;
//    delay_ms(200);  // Flash LED briefly
//    LED_ACTIVITY = 0;
//}
//
//#ifdef USE_VEML7700
//void initI2C(void) {
//    I2C1BRG = I2C_BAUD;
//    I2C1CONbits.I2CEN = 1;
//}
//
//void I2CStart(void) {
//    I2C1CONbits.SEN = 1;
//    while (I2C1CONbits.SEN);
//}
//
//void I2CStop(void) {
//    I2C1CONbits.PEN = 1;
//    while (I2C1CONbits.PEN);
//}
//
//void I2CSend(uint8_t data) {
//    I2C1TRN = data;
//    while (I2C1STATbits.TRSTAT);
//    while (I2C1STATbits.ACKSTAT);  // Wait for ACK
//}
//
//uint8_t I2CRead(bool ack) {
//    I2C1CONbits.RCEN = 1;
//    while (!I2C1STATbits.RBF);
//    uint8_t data = I2C1RCV;
//    I2C1CONbits.ACKDT = !ack;
//    I2C1CONbits.ACKEN = 1;
//    while (I2C1CONbits.ACKEN);
//    return data;
//}
//
//void VEML7700_Init(void) {
//    I2CStart();
//    I2CSend(VEML7700_ADDR | 0);  // Write
//    I2CSend(0x00);               // ALS_CONF register
//    I2CSend(0x00);
//    I2CSend(0x00);
//    I2CStop();
//}
//
//uint16_t VEML7700_ReadLuxRaw(void) {
//    I2CStart();
//    I2CSend(VEML7700_ADDR | 0);
//    I2CSend(0x04);  // ALS_DATA
//    I2CStop();
//
//    I2CStart();
//    I2CSend(VEML7700_ADDR | 1);  // Read
//    uint8_t lsb = I2CRead(true);
//    uint8_t msb = I2CRead(false);
//    I2CStop();
//
//    return (msb << 8) | lsb;
//}
//
//float calculateLux(uint16_t raw) {
//    return raw * 0.0576f;  // Default gain + integration time
//}
//#endif  // USE_VEML7700



////pir only
//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//
//// --- Timing Constants ---
////#define LIGHT_ON_TIME_MS 10000UL     // 10 seconds
//#define LIGHT_ON_TIME_MS 14000  // ~10 seconds actual time
//
//#define COOLDOWN_MS 2000         // 2 minutes
//#define LOOP_DELAY_MS 1000          // 1 second 
//
//// --- Pin Definitions (do not modify)
//#define PIR1_SENSOR_PIN PORTAbits.RA0     // PIR 1 input
//#define LED_POWER LATBbits.LATB7          // Power LED (always on)
//#define LED_ACTIVITY LATBbits.LATB2       // Activity LED (triggered by PIR)
//#define RS485_DE_RE_LAT LATBbits.LATB4    // DE/RE pin
//#define RS485_DE_RE_TRIS TRISBbits.TRISB4 // TRIS for DE/RE pin
//
//// --- Function Prototypes ---
//void initUART(void);
//void sendUARTMessage(const char *message);
//void initLEDs(void);
//void delay_ms(unsigned long ms);
//void initRS485(void);
//
//int main(void) {
//    AD1PCFGL = 0xFFFF;  // All analog pins digital
//
//    // Set directions
//    TRISAbits.TRISA0 = 1;   // PIR1 input
//    TRISBbits.TRISB2 = 0;   // LED_ACTIVITY output
//    TRISBbits.TRISB7 = 0;   // LED_POWER output
//    RS485_DE_RE_TRIS = 0;   // RS-485 DE/RE pin
//
//    // Peripheral Pin Select (PPS)
//    __builtin_write_OSCCONL(OSCCON & 0xBF);  // Unlock PPS
//    RPINR18bits.U1RXR = 14;                  // RX on RP14 (RB14)
//    RPOR5bits.RP11R = 3;                     // TX on RP11 (RB3)
//    __builtin_write_OSCCONL(OSCCON | 0x40);  // Lock PPS
//
//    initUART();
//    initLEDs();
//    initRS485();
//
//    sendUARTMessage("System initialized");
//
//    bool inCooldown = false;
//    unsigned long cooldownTimer = 0;
//
//    bool lightActive = false;
//    unsigned long lightTimer = 0;
//
//    while (1) {
//        // PIR Trigger
//        if (!inCooldown && PIR1_SENSOR_PIN == 1) {
//            sendUARTMessage("PIR 1 was activated");
//
//            LED_ACTIVITY = 1;
//            lightActive = true;
//            lightTimer = LIGHT_ON_TIME_MS;
//
//            inCooldown = true;
//            cooldownTimer = COOLDOWN_MS;
//        }
//
//        // Handle Activity LED timing
//        if (lightActive) {
//            if (lightTimer > 0) {
//                lightTimer -= LOOP_DELAY_MS;
//            } else {
//                LED_ACTIVITY = 0;
//                lightActive = false;
//            }
//        }
//
//        // Handle Cooldown
//        if (inCooldown) {
//            if (cooldownTimer > 0) {
//                cooldownTimer -= LOOP_DELAY_MS;
//            } else {
//                inCooldown = false;
//            }
//        }
//
//        delay_ms(LOOP_DELAY_MS);  // Single delay for timing accuracy
//    }
//
//    return 0;
//}
//
//void initUART(void) {
//    U1MODE = 0x8000;  // Enable UART
//    U1BRG = 8;        // 115200 baud @ 16MHz Fcy
//    U1STA = 0x0400;   // Enable TX
//}
//
//void initRS485(void) {
//    RS485_DE_RE_LAT = 0;  // Start in receive mode
//}
//
//void sendUARTMessage(const char *message) {
//    RS485_DE_RE_LAT = 1;  // Enable transmit
//
//    while (*message) {
//        while (U1STAbits.UTXBF);  // Wait if TX buffer full
//        U1TXREG = *message++;
//    }
//
//    while (!U1STAbits.TRMT);  // Wait for last char to send
//    U1TXREG = '\n';
//    while (!U1STAbits.TRMT);
//    delay_ms(1);
//
//    RS485_DE_RE_LAT = 0;  // Return to receive
//}
//
//void initLEDs(void) {
//    LED_POWER = 1;       // Power LED always on
//    LED_ACTIVITY = 0;    // Activity LED off at start
//}
//
//// ? Calibrated delay (adjust inner loop for accuracy)
//void delay_ms(unsigned long ms) {
//    while (ms--) {
//        for (volatile unsigned int i = 0; i < 250; i++);  // ~1ms @ 16 MIPS
//    }
//}
//



// PIR Only
#include "xc.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// --- Timing Constants ---
#define LOOP_DELAY_MS 50  // Loop runs every 50ms for debounce

// --- Pin Definitions ---
#define PIR1_SENSOR_PIN PORTAbits.RA0     // PIR 1 input
#define LED_POWER LATBbits.LATB7          // Power LED (always on)
#define LED_ACTIVITY LATBbits.LATB2       // Activity LED (triggered by PIR)
#define RS485_DE_RE_LAT LATBbits.LATB4    // DE/RE pin
#define RS485_DE_RE_TRIS TRISBbits.TRISB4 // TRIS for DE/RE pin

// --- Function Prototypes ---
void initUART(void);
void sendUARTMessage(const char *message);
void initLEDs(void);
void delay_ms(unsigned long ms);
void initRS485(void);
void blinkActivityLED(void);

int main(void) {
    AD1PCFGL = 0xFFFF;  // All analog pins digital

    // Set directions
    TRISAbits.TRISA0 = 1;   // PIR1 input
    TRISBbits.TRISB2 = 0;   // LED_ACTIVITY output
    TRISBbits.TRISB7 = 0;   // LED_POWER output
    RS485_DE_RE_TRIS = 0;   // RS-485 DE/RE pin

    // Peripheral Pin Select (PPS)
    __builtin_write_OSCCONL(OSCCON & 0xBF);  // Unlock PPS
    RPINR18bits.U1RXR = 14;                  // RX on RP14 (RB14)
    RPOR5bits.RP11R = 3;                     // TX on RP11 (RB3)
    __builtin_write_OSCCONL(OSCCON | 0x40);  // Lock PPS

    initUART();
    initLEDs();
    initRS485();

    sendUARTMessage("System initialized");

    bool inCooldown = false;
    unsigned long cooldownCounter = 0;

    while (1) {
        if (!inCooldown && PIR1_SENSOR_PIN == 1) {
            sendUARTMessage("PIR 1 was activated");
            blinkActivityLED();  // LED on 2s, off 1s
            inCooldown = true;
            cooldownCounter = 60;  // 60 x 50ms = 3000ms (3 seconds)
        }

        if (inCooldown) {
            if (cooldownCounter > 0) {
                cooldownCounter--;
            } else {
                inCooldown = false;
            }
        }

        delay_ms(LOOP_DELAY_MS);  // ~50ms loop
    }

    return 0;
}

void initUART(void) {
    U1MODE = 0x8000;  // Enable UART
    U1BRG = 8;        // 115200 baud @ 16MHz Fcy
    U1STA = 0x0400;   // Enable TX
}

void initRS485(void) {
    RS485_DE_RE_LAT = 0;  // Start in receive mode
}

void sendUARTMessage(const char *message) {
    RS485_DE_RE_LAT = 1;  // Enable transmit

    while (*message) {
        while (U1STAbits.UTXBF);  // Wait if TX buffer full
        U1TXREG = *message++;
    }

    while (!U1STAbits.TRMT);  // Wait for last char to send
    U1TXREG = '\n';
    while (!U1STAbits.TRMT);
    delay_ms(1);

    RS485_DE_RE_LAT = 0;  // Return to receive
}

void initLEDs(void) {
    LED_POWER = 1;       // Power LED always on
    LED_ACTIVITY = 0;    // Activity LED off at start
}

void blinkActivityLED(void) {
    LED_ACTIVITY = 1;
    delay_ms(2000);   // 2 seconds ON
    LED_ACTIVITY = 0;
    delay_ms(1000);   // 1 second OFF
}

// ~1ms delay at 16 MIPS
void delay_ms(unsigned long ms) {
    while (ms--) {
        for (volatile unsigned int i = 0; i < 250; i++);
    }
}
