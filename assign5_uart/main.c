#include <stdint.h>
#include <stdbool.h>
//#include "C:/ti/TivaWare_C_Series-2.2.0.295/inc/tm4c123gh6pm.h"
#include "inc/tm4c123gh6pm.h"


#define SOP 0x30  // 011 (Binary) shifted into position
#define EOP 0x60  // 110 (Binary) shifted into position
#define ACK 0x06  // ACK signal expected
#define SEQ 0x0  // Sequence number for device sending
#define DES 0x0   // Destination (sending or reciving)
#define CTRL 0x5  // Control code for packet
#define ADD 0x7 // Address of where the packet is going
#define CRC 0x2   // CRC

void UART_Init(void);
void UART_SendByte(uint8_t data);
uint8_t UART_ReceiveByte(void);
bool CheckForCollision(void);
bool WaitForACK(void);
void TransmitPacket(uint8_t payload);

int main(void) {
    UART_Init();
    uint8_t payload = 0x55;  // Example payload

    while (1) {
        if (!CheckForCollision()) {
            TransmitPacket(payload);
            if (WaitForACK()) {
                // Transmission successful
            } else {
                TransmitPacket(payload); // Retransmit
            }
        }
    }
}

void UART_Init(void) {
    SYSCTL_RCGCUART_R |= 0x01;  // Enable UART0
    SYSCTL_RCGCGPIO_R |= 0x01;  // Enable Port A

    GPIO_PORTA_AFSEL_R |= 0x03; // Enable alt function on PA0, PA1
    GPIO_PORTA_PCTL_R |= 0x11;  // Configure PA0, PA1 for UART
    GPIO_PORTA_DEN_R |= 0x03;   // Enable digital function

    UART0_CTL_R &= ~0x01;       // Disable UART0
    UART0_IBRD_R = 104;         // Baud rate 9600 (assuming 16MHz clock)
    UART0_FBRD_R = 11;
    UART0_LCRH_R = 0x60;        // 8-bit, no parity, 1-stop bit
    UART0_CC_R = 0x00;          // System clock
    UART0_CTL_R |= 0x301;       // Enable UART0
}

void UART_SendByte(uint8_t data) {
    while ((UART0_FR_R & 0x20) != 0);
    UART0_DR_R = data;
}

uint8_t UART_ReceiveByte(void) {
    while ((UART0_FR_R & 0x10) != 0);
    return (uint8_t)(UART0_DR_R & 0xFF);
}

bool CheckForCollision(void) {
    // Placeholder for collision detection logic
    return false;
}

bool WaitForACK(void) {
    uint8_t response = UART_ReceiveByte();
    return (response == ACK);
}

void TransmitPacket(uint8_t payload) {
    uint8_t packet[4];
    packet[0] = 0x80 | SOP | SEQ | DES;    // First byte starts with '1' followed by SoP, Sequence, and Destination
    packet[1] = 0x00 | CTRL | ADD;          // Second byte starts with '0'
    packet[2] = 0x00 | payload; // Third byte starts with '0', contains payload
    packet[3] = 0x80 | EOP | CRC;    // Fourth byte starts with '1'
    int i;
    for (i = 0; i < 4; i++) {
        UART_SendByte(packet[i]);
    }
}
