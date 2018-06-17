#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "utils/uartstdio.h"
#include "LIB/Mfrc522.h"
#include "tm4c123gh6pm.h"

/*PIN Connections:
 * Used SSI2 (Module 2)
 *
 * To use another Module or Reset Pin, the variables...
 * ...NRSTPD and chipSelectPin should be changed to the Pin Mask.
 *
 * Also, in Mfrc522.cpp, the definitions of CHIPSELECT_BASE,...
 * ...NRSTPD_BASE and SSI_BASE must be changed to the respective...
 * ...Port Base used.
 *
 * Finally, the respective chipSelectPin and NRSTPD...
 * ... GPIO Base and Pin should be enabled in InitSSI function.
 *
 *
 * Further versions should auto-change this values.
 *
 *
 * SDA / CS / FSS ------------ PB5
 * SCK  / CLK     ------------ PB4
 * MOSI / TX      ------------ PB7
 * MISO /  RX     ------------ PB6
 *
 * RST            ------------ PF0 *
 *
 */

#define redLED   0x00000002
#define blueLED  0x00000004
#define greenLED 0x00000008

#define CARD_LENGTH 10

void initLeds();
void dumpHex(unsigned char* buffer, int len);

int chipSelectPin = 0x20;  //PB5
int NRSTPD = 0x01; //PF0


uint8_t Version;
uint8_t AntennaGain;
uint8_t status;
uint32_t readTeste;
unsigned char str[MAX_LEN];
unsigned char cardID[CARD_LENGTH];

//Library modified to work with CCS
Mfrc522 Mfrc522(chipSelectPin, NRSTPD);

void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, 16000000);
}

void InitSSI(){
    uint32_t junkAuxVar;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //SDA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //reset

    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    //GPIOPinConfigure(GPIO_PB5_SSI2FSS);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);

    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 |
                   GPIO_PIN_7);

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5); //chipSelectPin
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0); //NRSTPD

    //
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 4000000, 8);
    //
    // Enable the SSI0 module.
    //
    SSIEnable(SSI2_BASE);

    while(SSIDataGetNonBlocking(SSI2_BASE, &junkAuxVar)){}

    UARTprintf("SSI Enabled! SPI Mode!  \nData: 8bits.\n\n");

}

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //40MHz

    InitConsole();
    initLeds();

    InitSSI();

    GPIOPinWrite(GPIO_PORTB_BASE, chipSelectPin, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, NRSTPD, NRSTPD);

    Mfrc522.Init();

    Version = Mfrc522.ReadReg(VersionReg);
    AntennaGain = Mfrc522.ReadReg(PICC_REQIDL) & (0x07<<4);

    UARTprintf("Version: '0x%x' \n", Version);
    UARTprintf("Antenna Gain: '0x%x' \n\n", AntennaGain);

    while(1){
        status = Mfrc522.Request(PICC_REQIDL, str);
        if(status == MI_OK){
            UARTprintf("Cartao Detectado! \n"); //Card Detected
            GPIOPinWrite(GPIO_PORTF_BASE, blueLED, blueLED);
        }

        status = Mfrc522.Anticoll(str);
        memcpy(cardID, str, 10);

        if(status == MI_OK){
            UARTprintf("ID: \n");
            dumpHex((unsigned char*)cardID, CARD_LENGTH);
            GPIOPinWrite(GPIO_PORTF_BASE, blueLED, 0);
            SysCtlDelay(SysCtlClockGet()/2); //Delay
        }
    }
}

void initLeds(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void dumpHex(unsigned char* buffer, int len){
    int i;

    UARTprintf(" ");
    for(i=0; i < len; i++) {
        UARTprintf("0x%x, ", buffer[i]);
    }
    UARTprintf("  FIM! \r\n"); //End
}

