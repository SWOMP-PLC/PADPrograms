#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"

#define SOP  0x30  // Start of Packet
#define EOP  0x60  // End of Packet
#define ACK  0x06  // ACK signal
#define SEQ  0x00  // Sequence number
#define DES  0x00  // Destination
#define CTRL 0x50  // Control field
#define ADD  0x07  // Address
#define POLYNOMIAL 0x3 // 4-bit CRC polynomial x^4 + x + 1
#define SW1 GPIO_PIN_4

void InitUART(void);
void InitConsole();
void UART_SendByte(uint8_t data);
void TransmitPacket(uint8_t payload);
bool WaitForAck(void);
char ReceivePacketByte(void);
uint8_t calculateCRC4(uint8_t divisor);
void InitButton(void);

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

        InitUART();
        InitConsole();
        InitButton();
        srand(SysCtlClockGet());

            UARTprintf("Console Set Complete. \n");
    while (1) {
        if ((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & GPIO_PIN_4) == 0) {  // Button pressed (logic low)
                    UARTprintf("Button Pressed. Transmitting...\n");

                    TransmitPacket(0x55);  // Send a packet

                    if (WaitForAck()) {
                        UARTprintf("Transmission confirmed.\n");
                    } else {
                        UARTprintf("ACK failed or corrupted. Retransmitting...\n");
                        uint32_t randomDelayMs = 100 + (rand() % 400);  // 100 to 500 ms
                        uint32_t delayCycles = (SysCtlClockGet() / 3000) * randomDelayMs;
                        SysCtlDelay(delayCycles);
                        TransmitPacket(0x55);
                    }

                    // Simple debounce delay
                    SysCtlDelay(SysCtlClockGet() / 10);  // ~100ms
                }
            }
}


uint8_t calculateCRC4(uint8_t divisor) {
    uint8_t crc = 0;
    divisor <<= 4;  // Shift data to make space for CRC calculation

    int i;  // Declare the loop counter outside the loop

    for (i = 7; i >= 0; i--) {  // Loop through 8 bits
        uint8_t bit = (divisor >> i) & 1;
        uint8_t crcMsb = (crc >> 3) & 1;
        crc <<= 1;

        if (bit ^ crcMsb) {
            crc ^= POLYNOMIAL;
        }

        crc &= 0xF;
    }

    return crc;
}


void InitUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);  // Enable UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  // Enable Port B for UART

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTFIFOEnable(UART1_BASE);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,
                         (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void InitConsole() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
                         (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTStdioConfig(0, 9600, SysCtlClockGet());
}

void InitButton(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);       // Enable Port F
    GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_4);        // Unlock PF4
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4); // Set PF4 as input
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // Enable pull-up
}

void UART_SendByte(uint8_t data) {
    while (UARTBusy(UART1_BASE));  // Wait until UART is ready
    UARTCharPut(UART1_BASE, data);
}

void TransmitPacket(uint8_t payload) {

        uint8_t packet[4];
        uint8_t crc = calculateCRC4(payload);
        int i; // Declare i here

        packet[0] = 0x80 | SOP | SEQ | DES;  // First byte
        packet[1] = 0x00 | CTRL | ADD;       // Second byte
        packet[2] = 0x00 | payload;          // Third byte
        packet[3] = 0x80 | EOP | crc;        // Fourth byte with CRC

        for (i = 0; i < 4; i++) {
            UART_SendByte(packet[i]);
        }
}


char ReceivePacketByte(void) {
    SysCtlDelay(SysCtlClockGet()*2 / 3);
    if(UARTCharsAvail(UART1_BASE)) {
        char data;
        data = UARTCharGet(UART1_BASE);
    return (unsigned char) data;
} else {
    return 0;
    }
}

bool WaitForAck(void) {
    uint8_t response = ReceivePacketByte();
    if (response == ACK) {

        return true;
    }else

    return false;
}

