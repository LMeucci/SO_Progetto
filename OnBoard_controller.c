#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*--------------------------------BIT MACROS-------------------------------------------------*/
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x) (0x01 << (x))
#define ZERO 0x00
/*--------------------------------------------------------------------------------------------*/

/*------------------------Blocking serial ops-------------------------------------------------*/
#define BAUD 19200
#define MYUBRR (F_CPU/16/BAUD-1)

void UART_init(void){
  // Set baud rate
  UBRR0H = (uint8_t)(MYUBRR>>8);
  UBRR0L = (uint8_t) MYUBRR;

  UCSR0C = BIT(UCSZ01) | BIT(UCSZ00);               /* 8-bit data */
  UCSR0B = BIT(RXEN0) | BIT(TXEN0) | BIT(RXCIE0);   /* Enable RX and TX */

}

void UART_putChar(uint8_t c){
  // wait for transmission completed, looping on status bit
  while ( !(UCSR0A & (1<<UDRE0)) );

  // Start transmission
  UDR0 = c;
}

uint8_t UART_getChar(void){
  // Wait for incoming data, looping on status bit
  while ( !(UCSR0A & (1<<RXC0)) );

  // Return the data
  return UDR0;

}

// reads a string until the first newline or 0
// returns the size read
uint8_t UART_getString(uint8_t* buf){
  uint8_t* b0=buf; //beginning of buffer
  while(1){
    uint8_t c=UART_getChar();
    *buf=c;
    ++buf;
    // reading a 0 terminates the string
    if (c==0)
      return buf-b0;
    // reading a \n  or a \r return results
    // in forcedly terminating the string
    if(c=='\n'||c=='\r'){
      *buf=0;
      ++buf;
      return buf-b0;
    }
  }
}

void UART_putString(uint8_t* buf){
  while(*buf){
    UART_putChar(*buf);
    ++buf;
  }
}
/*--------------------------------------------------------------------------------------------*/

/*---------------Interrupt driven check for a new device (pc) connected: Timer1 used----------*/
volatile uint8_t pc_connected_check=0;

ISR(TIMER1_COMPA_vect) {
  pc_connected_check= 1;
}
/*--------------------------------------------------------------------------------------------*/

uint8_t check_auth(uint8_t* buf){
  uint8_t* code= (uint8_t*)"codice";
  uint8_t valid_app= 1;
  int i;
  for(i=0; i<sizeof(buf);++i){
    if(*(buf+i)!= *(code+i)){
      valid_app -=1;
      break;
    }
  }
  return valid_app;
}

#define MAX_BUF 256

int main(void){
  UART_init();
  const int timer_duration_ms=2000;


  // configure timer: Prescaler=1024, Interrupt_delay= 2s. Interrupt checks if any PC was connected (incoming data)
  TCCR1A = 0;
  TCCR1B = BIT(WGM12) | BIT(CS10) | BIT(CS12);
  uint16_t ocrval= (uint16_t)(15.62*timer_duration_ms);
  OCR1A = ocrval;

  cli();
  TIMSK1 |= BIT(OCIE1A);  // enable the timer interrupt
  sei();

  uint8_t buf[MAX_BUF];   // Buffer to store incoming data
  uint8_t commands_enable= 0;
  while(1){
    if(pc_connected_check){
      UART_putString((uint8_t*)"Checking if pc is connected\n");

//while(1){                                                                     //DEBUG
      cli();
      UART_getString(buf);
      UART_putString((uint8_t*)"Data received: ");
      UART_putString(buf);

      UART_putString((uint8_t*)"Checking app authorization\n");
      if(check_auth(buf)){
        UART_putString((uint8_t*)"Code accepted\n");
        commands_enable=1;
      }
      else {
        UART_putString((uint8_t*)"Invalid code received\n");
      }
//}                                                                             //DEBUG
      pc_connected_check=0;
      sei();
    }

    if(commands_enable){
      UART_putString((uint8_t*)"Waiting for commands\n");
      // doing stuff...
      commands_enable= 0;
    }

  }
}





/*
TO DO:
1- Protocollo Handshake (manca hello inviato da Board a PC)
2- Binary + checksum com
3- Non blocking serial operations

*- Device commands

DONE:
- Timer to check if PC is connected
- Blocking serial operations using UART FUNCTIONS
-

*/
