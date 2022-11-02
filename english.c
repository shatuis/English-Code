/*
 * File:   PRUEBA_INT.c
 * Author: Jorge Gomez
 *
 * Created on 8 de agosto de 2016, 01:53 PM
 */

#include <xc.h>
#define _XTAL_FREQ 8000000
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has RB3 function, low voltage programming disabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include "lib_usart_pic16_v1.0/usart_pic16.h"

void USARTInit(uint16_t baud_rate)
{
    //Setup queue
    UQFront=UQEnd=-1;
        
    //SPBRG
    switch(baud_rate)
    {
     case 9600:
        SPBRG=0x33;
        break;
    }
    //TXSTA
    TXSTAbits.TX9=0;  //8 bit transmission
    TXSTAbits.TXEN=1; //Transmit enable
    TXSTAbits.SYNC=0; //Async mode
    TXSTAbits.BRGH=1; //High speed baud rate

    //RCSTA
    RCSTAbits.SPEN=1;   //Serial port enabled
    RCSTAbits.RX9=0;    //8 bit mode
    RCSTAbits.CREN=1;   //Enable receive
    RCSTAbits.ADDEN=0;  //Disable address detection

    BAUDCTLbits.BRG16 = 0;
    
    //Receive interrupt
    PIE1bits.RCIE=1;
    INTCONbits.PEIE=1;

    ei();
}

char UART_Read()
{
    PIE1bits.RCIE = 1;
  while(!RCIF);
    PIE1bits.RCIE = 0;
  return RCREG;
}

void Test()
{
 int d =0;
     while(d != 5)
     {
         PORTAbits.RA1 = 1;
         __delay_ms(100);
         PORTAbits.RA1 = 0;
         __delay_ms(100);
         d++;
     }
}


void USARTWriteChar(char ch)
{
  while(!PIR1bits.TXIF);

  TXREG=ch;
}

void writeEEPROM(unsigned char address, unsigned char datas)
{
  unsigned char INTCON_SAVE;//To save INTCON register value
  EEADR = address; //Address to write
  EEDATA = datas; //Data to write
  EECON1bits.EEPGD = 0; //Selecting EEPROM Data Memory
  EECON1bits.WREN = 1; //Enable writing of EEPROM
  INTCON_SAVE=INTCON;//Backup INCON interupt register
  INTCON=0; //Diables the interrupt
  EECON2=0x55; //Required sequence for write to internal EEPROM
  EECON2=0xAA; //Required sequence for write to internal EEPROM
  EECON1bits.WR = 1; //Initialise write cycle
  INTCON = INTCON_SAVE;//Enables Interrupt
  EECON1bits.WREN = 0; //To disable write
  while(PIR2bits.EEIF == 0)//Checking for complition of write operation
 {
   NOP(); //do nothing
 }
  PIR2bits.EEIF = 0; //Clearing EEIF bit
}

unsigned char readEEPROM(unsigned char address)
{
  EEADR = address; //Address to be read
  EECON1bits.EEPGD = 0;//Selecting EEPROM Data Memory
  EECON1bits.RD = 1; //Initialise read cycle
  return EEDATA; //Returning data
}

void main(void) {
      OSCCONbits.IRCF = 0b111;
 TRISB = 0x00;
 TRISD = 0x00;
 PORTD = 0x00;
    PORTB = 0x00;
     TRISA = 0x00;
     PORTA = 0x00;
     Test();
     USARTInit(9600);
      char pwd[] = {0x41,0x42,0x43,0x44,0x45,0x46};
      int cont = 0;
      int cont1 = 0;
     while(1)
     { 
         if(readEEPROM(0x8E) == 0xFF)
         {
             int va2 = 0;
             while(va2 == 0)
             {
                  int va = 0;
         int i = 0;
         for(i ; i <= 5 ; i++)
         {
             PORTAbits.RA1 = 1;
             __delay_ms(100);
             PORTAbits.RA1 = 0;
             if(pwd[i] == UART_Read())
             {
                 va++;
             }
             else
             {
                 i = 5;
                 USARTWriteChar(0x40);
             }
         }
         if(va == 6)
         {
            USARTWriteChar(0x3F);
            char addresW = 0x00;
             char addresW1 = 0x34;
            int v1 = 0;
            char y = 0x00;
            while(v1 == 0)
            {
                y = UART_Read();
             writeEEPROM(addresW , y);
        
            addresW++;
            cont++;
            if(UART_Read() == 0x3F)
            {
                v1 = 1;
            }
            }
            for(int j=0 ; j < cont ; j++)
            {
             USARTWriteChar(readEEPROM(j));
              PORTAbits.RA1 = 1;
             __delay_ms(100);
             PORTAbits.RA1 = 0;
            }
            v1 = 0;
             while(v1 == 0)
            {
            y = UART_Read();
            writeEEPROM(addresW1 , y);
            addresW1++;
            cont1++;
            if(UART_Read() == 0x3F)
            {
            v1 = 1;
            }
            }
            for(int k=52 ; k < 59 ; k++)
            {
            USARTWriteChar(readEEPROM(k));
            PORTAbits.RA1 = 1;
            __delay_ms(100);
            PORTAbits.RA1 = 0;
            }
            va2 = 1;
            }
             }
         writeEEPROM(0x8E , 0x77);
         writeEEPROM(0x8F , cont);
         }
         USARTWriteChar(0x20);
         if(UART_Read() == 0x3F)
         {
        for(int u=0 ; u < readEEPROM(0x8F) ; u++)
            {
             USARTWriteChar(readEEPROM(u));
              PORTAbits.RA1 = 1;
             __delay_ms(100);
             PORTAbits.RA1 = 0;
            }
        USARTWriteChar(0x20);
        for(int w=52 ; w < 59 ; w++)
            {
            USARTWriteChar(readEEPROM(w));
            PORTAbits.RA1 = 1;
            __delay_ms(100);
            PORTAbits.RA1 = 0;
         }
         }
     }
    return;
}