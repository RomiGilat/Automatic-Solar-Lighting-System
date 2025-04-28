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


///above is the blinking back and forth code


//now lets do the sensing PIR for 1 & 2

///test ra0 

// tested good,

//test ra1

//
//
//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//
//// Pin definitions
//#define PIR_SENSOR_PIN PORTAbits.RA1   // PIR sensor (RA1)
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
//    TRISAbits.TRISA1 = 1;  // PIR input
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






////---- pir 1 & 2------// 

//
//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//
//// --- Pin Definitions ---
//#define PIR1_SENSOR_PIN PORTAbits.RA0  // PIR 1 (RA0)
//#define PIR2_SENSOR_PIN PORTAbits.RA1  // PIR 2 (RA1)
//
//#define LED_POWER LATBbits.LATB7       // Power LED (RB7)
//#define LED_ACTIVITY LATBbits.LATB2    // Activity LED (RB2)
//
//// --- Function Prototypes ---
//void initUART(void);
//void sendUARTMessage(const char *message);
//void initLEDs(void);
//void delay_ms(unsigned int ms);
//void blinkActivityLED(void);
//
//int main(void) {
//    // Set all analog-capable pins to digital
//    AD1PCFGL = 0xFFFF;
//
//    // Configure pin directions
//    TRISAbits.TRISA0 = 1;  // PIR1 input
//    TRISAbits.TRISA1 = 1;  // PIR2 input
//    TRISBbits.TRISB2 = 0;  // Activity LED output
//    TRISBbits.TRISB7 = 0;  // Power LED output
//
//    // PPS: UART RX on RP14 (RB14), TX on RP11 (RB3)
//    __builtin_write_OSCCONL(OSCCON & 0xBF);  // Unlock PPS
//    RPINR18bits.U1RXR = 14;
//    RPOR5bits.RP11R = 3;
//    __builtin_write_OSCCONL(OSCCON | 0x40);  // Lock PPS
//
//    // Init peripherals
//    initUART();
//    initLEDs();
//    sendUARTMessage("System initialized");
//
//    while (1) {
//        if (PIR1_SENSOR_PIN == 1 || PIR2_SENSOR_PIN == 1) {
//            if (PIR1_SENSOR_PIN == 1) {
//                sendUARTMessage("PIR 1 was activated");
//            }
//            if (PIR2_SENSOR_PIN == 1) {
//                sendUARTMessage("PIR 2 was activated");
//            }
//
//            blinkActivityLED();     // 2 sec ON, 1 sec cooldown
//        }
//    }
//
//    return 0;
//}
//
//// --- Initialize UART ---
//void initUART(void) {
//    U1MODE = 0x8000;  // Enable UART
//    U1BRG = 8;        // Baud rate (115200 @ 16 MHz)
//    U1STA = 0x0400;   // Enable TX
//}
//
//// --- Send string via UART with newline ---
//void sendUARTMessage(const char *message) {
//    while (*message) {
//        while (U1STAbits.UTXBF);
//        U1TXREG = *message++;
//    }
//    while (!U1STAbits.TRMT);
//    U1TXREG = '\n';
//}
//
//// --- LED initialization ---
//void initLEDs(void) {
//    LED_POWER = 1;     // Power LED always ON
//    LED_ACTIVITY = 0;  // Activity LED initially OFF
//}
//
//// --- Delay function (~1ms per iteration at 16MHz Fcy) ---
//void delay_ms(unsigned int ms) {
//    while (ms--) {
//        for (volatile unsigned int i = 0; i < 1600; i++);
//    }
//}
//
//// --- Blink LED for 2 seconds, then cooldown 1 second ---
//void blinkActivityLED(void) {
//    LED_ACTIVITY = 1;
//    delay_ms(2000);       // ON for 2 seconds
//    LED_ACTIVITY = 0;
//    delay_ms(1000);       // Cooldown for 1 second
//}
//
//
//

//
//#include "xc.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//
//// --- Pin Definitions ---
//#define PIR1_SENSOR_PIN PORTAbits.RA0   // PIR 1 input
//#define PIR2_SENSOR_PIN PORTAbits.RA1   // PIR 2 input
//#define LED_POWER LATBbits.LATB7        // Power LED
//#define LED_ACTIVITY LATBbits.LATB2     // Activity LED (LED1)
//
//// --- Function Prototypes ---
//void initUART(void);
//void sendUARTMessage(const char *message);
//void initLEDs(void);
//void delay_ms(unsigned int ms);
//void blinkActivityLED(void);
//
//int main(void) {
//    AD1PCFGL = 0xFFFF;  // Make all analog-capable pins digital
//
//    // Set pin directions
//    TRISAbits.TRISA0 = 1;  // PIR1 input
//    TRISAbits.TRISA1 = 1;  // PIR2 input
//    TRISBbits.TRISB2 = 0;  // Activity LED output
//    TRISBbits.TRISB7 = 0;  // Power LED output
//
//    // PPS config for UART1: TX on RP11 (RB3), RX on RP14 (RB14)
//    __builtin_write_OSCCONL(OSCCON & 0xBF);  // Unlock PPS
//    RPINR18bits.U1RXR = 14;
//    RPOR5bits.RP11R = 3;
//    __builtin_write_OSCCONL(OSCCON | 0x40);  // Lock PPS
//
//    initUART();
//    initLEDs();
//    sendUARTMessage("System initialized");
//
//    while (1) {
//        if (PIR1_SENSOR_PIN == 1) {
//            sendUARTMessage("PIR 1 was activated");
//            blinkActivityLED();
//        }
//
//        if (PIR2_SENSOR_PIN == 1) {
//            sendUARTMessage("PIR 2 was activated");
//            blinkActivityLED();
//        }
//    }
//
//    return 0;
//}
//
//// --- UART Initialization ---
//void initUART(void) {
//    U1MODE = 0x8000;  // Enable UART
//    U1BRG = 8;        // 115200 baud at 16MHz
//    U1STA = 0x0400;   // Enable TX
//}
//
//// --- Send string via UART ---
//void sendUARTMessage(const char *message) {
//    while (*message) {
//        while (U1STAbits.UTXBF);  // Wait if TX buffer full
//        U1TXREG = *message++;
//    }
//    while (!U1STAbits.TRMT);
//    U1TXREG = '\n';
//}
//
//// --- Initialize LEDs ---
//void initLEDs(void) {
//    LED_POWER = 1;      // Power LED ON
//    LED_ACTIVITY = 0;   // Activity LED OFF
//}
//
//// --- Delay function (~1ms per unit @ 16MHz) ---
//void delay_ms(unsigned int ms) {
//    while (ms--) {
//        for (volatile unsigned int i = 0; i < 1600; i++);
//    }
//}
//
//// --- Blink activity LED for 2s, then cooldown 1s ---
//void blinkActivityLED(void) {
//    LED_ACTIVITY = 1;
//    delay_ms(2000);      // LED ON 2 seconds
//    LED_ACTIVITY = 0;
//    delay_ms(1000);      // Cooldown 1 second
//}



#include "xc.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// --- Pin Definitions ---
#define PIR1_SENSOR_PIN PORTAbits.RA0     // PIR 1 input
#define PIR2_SENSOR_PIN PORTAbits.RA1     // PIR 2 input
#define LED_POWER LATBbits.LATB7          // Power LED
#define LED_ACTIVITY LATBbits.LATB2       // Activity LED
#define RS485_DE_RE_LAT LATBbits.LATB4    // DE/RE pin
#define RS485_DE_RE_TRIS TRISBbits.TRISB4 // TRIS for DE/RE pin

// --- Function Prototypes ---
void initUART(void);
void sendUARTMessage(const char *message);
void initLEDs(void);
void delay_ms(unsigned int ms);
void blinkActivityLED(void);
void initRS485(void);

int main(void) {
    AD1PCFGL = 0xFFFF;  // Make all analog-capable pins digital

    // Set directions
    TRISAbits.TRISA0 = 1;  // PIR1 input
    TRISAbits.TRISA1 = 1;  // PIR2 input
    TRISBbits.TRISB2 = 0;  // LED output
    TRISBbits.TRISB7 = 0;  // Power LED
    RS485_DE_RE_TRIS = 0;  // DE/RE pin as output

    __builtin_write_OSCCONL(OSCCON & 0xBF);  // Unlock PPS
    RPINR18bits.U1RXR = 14;                  // RX on RP14 (RB14)
    RPOR5bits.RP11R = 3;                     // TX on RP11 (RB3)
    __builtin_write_OSCCONL(OSCCON | 0x40);  // Lock PPS

    initUART();
    initLEDs();
    initRS485();
    sendUARTMessage("System initialized");

    while (1) {
        if (PIR1_SENSOR_PIN == 1) {
            sendUARTMessage("PIR 1 was activated");
            blinkActivityLED();
        }

        if (PIR2_SENSOR_PIN == 1) {
            sendUARTMessage("PIR 2 was activated");
            blinkActivityLED();
        }
    }

    return 0;
}

void initUART(void) {
    U1MODE = 0x8000;  // Enable UART
    U1BRG = 8;        // 115200 baud for 16MHz Fcy
    U1STA = 0x0400;   // Enable TX
}

void initRS485(void) {
    RS485_DE_RE_LAT = 0;  // Start in receive mode (safe default)
}

void sendUARTMessage(const char *message) {
    RS485_DE_RE_LAT = 1;  // ? Enable transmitter

    while (*message) {
        while (U1STAbits.UTXBF);  // Wait if TX buffer full
        U1TXREG = *message++;
    }

    while (!U1STAbits.TRMT);  // Wait for TX to complete
    U1TXREG = '\n';

    while (!U1STAbits.TRMT);  // Ensure '\n' finishes
    delay_ms(1);              // Brief delay to ensure line settles
    RS485_DE_RE_LAT = 0;      // ? Back to receive (if needed)
}

void initLEDs(void) {
    LED_POWER = 1;
    LED_ACTIVITY = 0;
}

void delay_ms(unsigned int ms) {
    while (ms--) {
        for (volatile unsigned int i = 0; i < 1600; i++);
    }
}

void blinkActivityLED(void) {
    LED_ACTIVITY = 1;
    delay_ms(2000);   // 2 seconds
    LED_ACTIVITY = 0;
    delay_ms(1000);   // 1 second cooldown
}
