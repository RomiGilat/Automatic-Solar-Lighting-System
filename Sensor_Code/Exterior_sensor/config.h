#ifndef CONFIG_H
#define CONFIG_H

// Boot Segment
#pragma config BWRP = WRPROTECT_OFF      // Boot Segment Write Protect
#pragma config BSS = NO_FLASH            // No boot flash segment
#pragma config RBS = NO_RAM              // No boot RAM segment

// Secure Segment
#pragma config SWRP = WRPROTECT_OFF      // Secure Segment Program Write Protect
#pragma config SSS = NO_FLASH            // No secure flash segment
#pragma config RSS = NO_RAM              // No secure RAM segment

// General Segment
#pragma config GWRP = OFF                // General Code Segment Write Protect

// Oscillator Settings
#pragma config FNOSC = FRC               // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = OFF                // Two-speed Startup disabled

// Clock
#pragma config POSCMD = EC               // Primary Oscillator Mode
#pragma config OSCIOFNC = ON             // OSC2 as digital I/O
#pragma config IOL1WAY = OFF             // Multiple PPS reconfig allowed
#pragma config FCKSM = CSECME            // Clock Switching & Fail-safe Clock Monitor enabled

// Watchdog Timer
#pragma config WDTPOST = PS32768         // WDT postscaler
#pragma config WDTPRE = PR128            // WDT prescaler
#pragma config WINDIS = OFF              // WDT in non-window mode
#pragma config FWDTEN = OFF              // WDT disabled

// Power-on Reset
#pragma config FPWRT = PWR1              // POR Timer
#pragma config ALTI2C = ON               // Alternate I2C pins

// Debug/Programming
#pragma config ICS = PGD1                // ICD/ICSP uses PGD1/PGC1
#pragma config JTAGEN = OFF              // JTAG disabled

#endif // CONFIG_H
