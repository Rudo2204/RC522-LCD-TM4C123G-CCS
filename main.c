#include <stdio.h>
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
#include "LIB/lcd.h"

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
 * ----------LCD SECTION -------------
 * VSS -> GND
 * VDD -> 5v
 * V0 -> middle potentiometer
 * RS -> PE0
 * RW -> GND
 * EN -> PE1
 * D4 -> PB0 (PD0)
 * D5 -> PB1 (PD1)
 * D6 -> PC4 (PD2)
 * D7 -> PC7 (PD3)
 * A -> 5v
 * K -> GND
 */

#define redLED   0x00000002
#define blueLED  0x00000004
#define greenLED 0x00000008

#define CARD_LENGTH 5 // Default should be 10

void initLeds();
void dumpHex(unsigned char* buffer, int len);
void lcd_default();
void lcd_print_access(unsigned char* buffer, int len);

int chipSelectPin = 0x20;  //PB5
int NRSTPD = 0x01; //PF0

uint8_t Version;
uint8_t AntennaGain;
uint8_t status;
uint32_t readTeste;
unsigned char str[MAX_LEN];
unsigned char cardID[CARD_LENGTH];
char master_card[CARD_LENGTH] = {0x49, 0x42, 0xb6, 0x99, 0x24};

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

    lcd_init();
    lcd_default();

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
            UARTprintf("Card Detected! \n"); //Card Detected
            GPIOPinWrite(GPIO_PORTF_BASE, blueLED, blueLED);
        }

        status = Mfrc522.Anticoll(str);
        //lcd_clear();
        //lcd_gotoxy(0, 1);
        //lcd_puts(str);
        memcpy(cardID, str, 5);

        if(status == MI_OK){
            UARTprintf("ID: ");
            dumpHex((unsigned char*)cardID, CARD_LENGTH);
            GPIOPinWrite(GPIO_PORTF_BASE, blueLED, 0);
            lcd_print_access((unsigned char*)cardID, CARD_LENGTH);
            //for(int i_tmp=0, i_tmp < 10; i_tmp++)
            //{
            //    if(!temp[i_tmp])
            //        break;
            //    else
            //        lcd_putc(temp[i_tmp]);
            //}
            SysCtlDelay(10000000); //Delay 1s
            lcd_default();
        }
    }
}

void initLeds(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void dumpHex(unsigned char* buffer, int len){
    int i;

    for(i=0; i < len; i++) {
        UARTprintf("%x ", buffer[i]);
        //lcd_puts(buffer[i]);
    }
    UARTprintf("    END!\n\n"); //End
}

void lcd_default(void) {
    lcd_clear();
    lcd_gotoxy(1,0);
    lcd_puts("Access Control");
    lcd_gotoxy(0,1);
    lcd_puts("Scan Your Card>>");
}

void lcd_print_access(unsigned char* buffer, int len) {
    int i;
    bool enable_access = true;
    lcd_clear();
    lcd_gotoxy(1, 1);
    lcd_puts("ID: ");
    for(i=0; i < len; i++) {
        if(buffer[i] != master_card[i])
            enable_access = false;
        if((buffer[i]/16) < 10)
            lcd_put_byte(1, buffer[i]/16 + 48);
        else
            lcd_put_byte(1, buffer[i]/16 + 55);
        if((buffer[i]%16) < 10)
            lcd_put_byte(1, buffer[i]%16 + 48);
        else
            lcd_put_byte(1, buffer[i]%16 + 55);
    }
    lcd_gotoxy(1, 0);
    if(enable_access)
        lcd_puts("Access Granted!");
    else
        lcd_puts("Access Denied!");

}
