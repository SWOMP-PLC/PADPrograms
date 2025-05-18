#include <stdint.h>
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
#define CRC  0x02  // CRC value

void InitUART(void);
void InitConsole();
void UART_SendByte(uint8_t data);
void TransmitPacket(uint8_t payload);
bool WaitForAck(void);
char ReceivePacketByte(void);

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

        InitUART();
        InitConsole();

            UARTprintf("Console Set Complete. \n");
    while (1) {
        TransmitPacket(0x55);  // Send a packet
        // Check for ACK
        if (WaitForAck()) {
                    UARTprintf("Transmission confirmed.\n");
                } else {
                    UARTprintf("ACK failed or corrupted.\n");
                }

        SysCtlDelay(SysCtlClockGet()*10 / 3);  // ~10-second delay
    }
}
// UART Initialization Function (Fixed for PB0/PB1)
void InitUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);  // Enable UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  // Enable Port B for UART

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTFIFOEnable(UART1_BASE);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}
void InitConsole() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(
            UART0_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    //Configure UART to use stdio libraries printf
    UARTStdioConfig(0, 115200, SysCtlClockGet());
}

// Function to send a single byte over UART
void UART_SendByte(uint8_t data) {
    while (UARTBusy(UART1_BASE));  // Wait until UART is ready
    UARTCharPut(UART1_BASE, data);
//    UARTCharPutNonBlocking(UART1_BASE, data);
}

// Function to transmit a 4-byte packet
void TransmitPacket(uint8_t payload) {
    uint8_t packet[4];
    int i;  // Declare i before the loop

    packet[0] = 0x80 | SOP | SEQ | DES;  // First byte
    packet[1] = 0x00 | CTRL | ADD;       // Second byte
    packet[2] = 0x00 | payload;          // Third byte
    packet[3] = 0x80 | EOP | CRC;        // Fourth byte


    for (i = 0; i < 4; i++) {  // Use the declared i
        UART_SendByte(packet[i]);
//        UARTprintf("Packet: %x \n", packet[i]);
    }
}

char ReceivePacketByte(void){
    char data;
    data = UARTCharGet(UART1_BASE);
    return (unsigned char) data;
}

bool WaitForAck(void) {
    uint8_t response = ReceivePacketByte();  // blocking receive
    if (response == ACK) {
        UARTprintf("ACK received.\n");
        return true;
    } else {
        UARTprintf("No ACK or incorrect response: %x\n", response);
        return false;
    }
}


