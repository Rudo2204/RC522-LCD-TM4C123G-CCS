#include "lcd.h"

 /****************************************************************************
* MODULE PRIVATE FUNCTIONS:
* void lcd_delay_us (unsigned long t);
* void lcd_delay_ms (unsigned long t);
* void lcd_put_byte(unsigned char rs, unsigned char data);
*
******************************************************************************/

//*****************************************************************************
// Delay functions
//*****************************************************************************
void lcd_delay_us (unsigned long t)
{
 // int index;
  while (t--)
  {
      //SysCtlDelay(SysCtlClockGet()/6000000);
      SysCtlDelay(50);
  }
}
void lcd_delay_ms (unsigned long t)
{
//  int index;
  while (t--)
  {
      SysCtlDelay(50000);
      //SysCtlDelay(SysCtlClockGet()/6000);
  }
}
//*****************************************************************************
// Send a byte of data (rs == 1) or command (rs == 0) to LCD
//*****************************************************************************
void lcd_put_byte(unsigned char rs, unsigned char data)
{
    LCD_RS_0;
    if(rs) LCD_RS_1;
    lcd_delay_us(20);
    LCD_EN_0;

    // send the high nibble
    if (data&BIT4)
        {LCD_DATA_4_1;}
        else
        {LCD_DATA_4_0;}
    if (data&BIT5)
        {LCD_DATA_5_1;}
        else
        {LCD_DATA_5_0;}
    if (data&BIT6)
        {LCD_DATA_6_1;}
        else
        {LCD_DATA_6_0;}
    if (data&BIT7)
        {LCD_DATA_7_1;}
        else
        {LCD_DATA_7_0;}

    lcd_delay_us(20);
    LCD_EN_1;
    lcd_delay_us(20);
    LCD_EN_0;

    // send the low nibble
    if (data&BIT0)
    {LCD_DATA_4_1;}
    else
    {LCD_DATA_4_0;}
    if (data&BIT1)
    {LCD_DATA_5_1;}
    else
    {LCD_DATA_5_0;}
    if (data&BIT2)
    {LCD_DATA_6_1;}
    else
    {LCD_DATA_6_0;}
    if (data&BIT3)
    {LCD_DATA_7_1;}
    else
    {LCD_DATA_7_0;}

    lcd_delay_us(20);
    LCD_EN_1;
    lcd_delay_us(20);
    LCD_EN_0;
}

 /****************************************************************************
* MODULE PUPLIC FUNCTIONS:
* void lcd_init(void)
* void lcd_clear(void)
* void lcd_gotoxy(unsigned char col, unsigned char row)
* void lcd_putc(char c)
* void lcd_puts(const char* s)
* void lcd_clr_puts (const char* s1 , const char* s2)
* void lcd_put_num (unsigned long val, char dec, unsigned char neg)
******************************************************************************/

//*****************************************************************************
// Initialization for LCD module
//*****************************************************************************
void lcd_init(void)
{
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

            // Set all signal pins as output
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
            GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);

            GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
            GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
            GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
            GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
    LCD_RS_0;
    LCD_EN_0;
    lcd_delay_ms(200);                // delay for power on

    lcd_delay_ms(2);     // wait for LCD
    lcd_put_byte(0,FOUR_BIT & LINES_5X7);            // Set LCD type
    lcd_delay_ms(2);     // wait for LCD

    lcd_put_byte(0,DOFF&CURSOR_OFF&BLINK_OFF);        // display off
    lcd_delay_ms(2);     // wait for LCD
    lcd_put_byte(0,DON&CURSOR_OFF&BLINK_OFF);        // display on
    lcd_delay_ms(2);     // wait for LCD

    lcd_put_byte(0,0x01);              // clear display and move cursor to home
    lcd_delay_ms(2);     // wait for LCD
    lcd_put_byte(0,SHIFT_CUR_LEFT);                // cursor shift mode
    lcd_delay_ms(2);     // wait for LCD
    lcd_put_byte(0,0x01);              // clear display and move cursor to home
    lcd_delay_ms(2);     // wait for LCD
}

//*****************************************************************************
// Clear the LCD
//*****************************************************************************
void lcd_clear(void)
{
    lcd_put_byte(0,0x01);        // display off
    lcd_delay_ms(2);     // wait for LCD
}

//*****************************************************************************
// Move the pointer of LCD to coordinate (column, row)
// Note that cursor is at (0,0) after reset of clear LCD
//*****************************************************************************
void lcd_gotoxy(unsigned char col, unsigned char row)
{
    unsigned char address;

    if(row!=0)
        address=0x40;
    else
        address=0;

    address += col;
    lcd_put_byte(0,0x80|address);
    lcd_delay_ms(2);     // wait for LCD
}

//*****************************************************************************
// Put a writable character on LCD
// If the character is '\f', clear the LCD
// If the character is '\n', move cursor to the second line
//*****************************************************************************
void lcd_putc(char c)
{
    switch(c){
        case '\f':
            lcd_put_byte(0, 0x01);
            lcd_delay_ms(2);     // wait for LCD
            break;
        case '\n':
            lcd_gotoxy(0, 0x01);
            break;
        default:
            lcd_put_byte(1, c);
            lcd_delay_ms(2);     // wait for LCD
            break;
    }
}

//*****************************************************************************
// Put a string s[] on LCD.
// Please note that the string is ended with '/0' (or 0x00)
//*****************************************************************************
void lcd_puts(const char* s)
{
    while(*s){
        lcd_putc(*s++);
    }
}

//*****************************************************************************
// Clear LCD. Put string s1[] on line 1 and string s2[] on line 2
//*****************************************************************************
void lcd_clr_puts (const char* s1 , const char* s2)
{
    lcd_clear ();
    lcd_puts (s1);
    lcd_gotoxy (0,1);
    lcd_puts (s2);
}

//*****************************************************************************
// Put a constant or a number stored in 'val' on LCD
// The number must be multiplied and convert in type "unsigned long"
//                                                or "unsigned int"
//   dec: amount of digits after '.'
//   neg: sign of the number, neg = 0 if the number is positive
//
// Example 1: lcd_put_num(val, 3, 0); // val = 12345
//        will put "12.345" on the LCD
// Example 2: lcd_put_num(val, 6, 1); // val = 12345
//        will put "-0.012345" on the LCD
//
// Note: This function may contain some bugs. If you can detect and fix them
//       , please contact me :d
//*****************************************************************************
void lcd_put_num (unsigned long val, char dec, unsigned char neg)
{
    char i, j, digit,k;
    long total;
    long temp;
    for (i = 0, total = val; total > 0;i++) // count number
        total /= 10;
        total = i;
    if (neg !=0 )
      lcd_putc ('-');
    if ((total - dec) <=0)    // if total < dec put 0.xxx
        lcd_putc('0');
    else
    {
      for (i=0; i< (total-dec); i++)
      {
        temp = 1;
        for (j=1;j<(total-i);j++)
            temp *=10;
        digit = (val/temp)%10;
        lcd_putc(digit + 0x30);
      }
    }
    if (dec > 0)
    {
      lcd_putc('.');
      if ( (dec - total)> 0)    // ex: val = 55; dec = 3 put 0.055
      {
        for ( i = 0; i < (dec-total);i++)
          lcd_putc('0');
        k = 0;
      }
      else
        k = total - dec;
      for (i=k; i< total; i++)
      {
       temp = 1;
       for (j=1;j<(total-i);j++)
           temp *=10;
       digit = (val/temp)%10;
       lcd_putc(digit + 0x30);
      }
    }
}

/******************************************************************************
 * END OF LCD.C
*******************************************************************************/
