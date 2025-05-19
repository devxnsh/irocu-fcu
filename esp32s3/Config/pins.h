#ifndef PINS_H
#define PINS_H

// Motor ESC Pins
#define ESC_PIN_1 21    // ESC 1 Pin (Motor 1)
#define ESC_PIN_2 22    // ESC 2 Pin (Motor 2)
#define ESC_PIN_3 23    // ESC 3 Pin (Motor 3)
#define ESC_PIN_4 24    // ESC 4 Pin (Motor 4)

#define I2C_SDA 8
#define I2C_SCL 9
#define BMP390_INT 10         // Interrupt for BMP390

// Communication Pins (if needed)
#define TELEM_TX 49    // Pin for receiving data from telemetry
#define TELEM_RX 50    // Pin for transmitting data
#define TELEM_CTS 52  //UART CTS PIN
#define TELEM_RTS 51  //UART RTS PIN

#endif // PINS_H
