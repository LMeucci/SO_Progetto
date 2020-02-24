#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define USART_BAUDRATE 19200
#define UBRR_VALUE (F_CPU/16/USART_BAUDRATE-1)

#define MAX_READ 32
#define MAX_WRITE 2

#define PIN_2 0
#define PIN_3 1
#define PIN_5 2
#define PIN_6 3
#define PIN_7 4
#define PIN_8 5
#define PIN_46 6
#define PIN_44 7

#define SIZECTL_MASK 0x07
#define TO_INT_MASK 0x30
#define SWITCH_MASK1 0xff
#define BUTTON_STEP 26

#define CMD_START 0x01
#define CMD_ASSIGN 0x02
#define CMD_READINGS 0x03
#define CMD_LED_SETUP 0x04
#define CMD_TEMP_SETUP 0x05

#define RSP_ACK 0x41
#define RSP_NACK 0x4E
#define RSP_UCMD 0x55
#define RSP_UPORT 0x50

/*--------------------------------All packet related things----------------------------------*/
typedef struct{
  uint8_t buffer[MAX_READ];   /* Payload buffer */
  uint8_t index;              /* index to navigate buffer, points to the next empty location */
}Packet;

/* Packet declaration */
Packet pck;
/* Response packets */
Packet rsp;

/* variables used to handle pwm and timer5      ----> [NOT IN USE] <-----
const uint16_t timer_duration_ms= 500;*/
uint8_t brightnessLED[8]={0};
//volatile uint8_t blinker= 0;


/* pins used to handle switches */
volatile uint8_t current_pins;

/* Packets initialization(erasing) */
void PckInit(Packet *pck)
{
  pck->index=0;    /* starting empty buffer location */
}

void RspInit(Packet *rsp)
{
  rsp->index=0;
  rsp->buffer[0]= RSP_NACK;   /* NAck response stored in the first byte */
  rsp->buffer[1]= RSP_NACK;   /* Only one field so checksum is just its copy */
}

void RspAck(Packet *rsp)
{
  rsp->buffer[0]= RSP_ACK;   /* Ack response */
  rsp->buffer[1]= RSP_ACK;
}

void RspUcmd(Packet *rsp)
{
  rsp->buffer[0]= RSP_UCMD;   /* Unknown command response */
  rsp->buffer[1]= RSP_UCMD;
}

void RspUport(Packet *rsp)
{
  rsp->buffer[0]= RSP_UPORT;  /* Unknown port response */
  rsp->buffer[1]= RSP_UPORT;
}

/* setup pwm using timer1 */
void pwmInit(uint8_t pin, uint8_t brightness)
{
  /*---------pin mapping:-------------
                         0=pin2, 1=pin3, 2=pin5,  3=pin6,
                         4=pin7, 5=pin8, 6=pin46, 7=pin44*/
  brightness= 255 -brightness; // need to invert user input because brightness=255 -> led off
  *(brightnessLED+(pin-TO_INT_MASK))= brightness;
  switch(pin-TO_INT_MASK)
  {
    case PIN_2:
      /* timer3, channel B (pin2) Fast PWM, non inverted, 8bit */
      TCCR3A |= (1<<COM3B1) | (1<<COM3B0) | (1<<WGM30);
      TCCR3B |= (1<<WGM32) | (1<<CS30);  // no prescaler
      DDRE |= (1<<PE4);                  // output pin set
      OCR3BH= 0;
      OCR3BL= brightness;
      break;
    case PIN_3:
      /* timer3, channel C (pin3) Fast PWM, non inverted, 8bit */
      TCCR3A |= (1<<COM3C1) | (1<<COM3C0) | (1<<WGM30);
      TCCR3B |= (1<<WGM32) | (1<<CS30);  // no prescaler
      DDRE |= (1<<PE5);                  // output pin set
      OCR3CH= 0;
      OCR3CL= brightness;
      break;
    case PIN_5:
      /* timer3, channel A (pin5) Fast PWM, non inverted, 8bit */
      TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM30);
      TCCR3B |= (1<<WGM32) | (1<<CS30);  // no prescaler
      DDRE |= (1<<PE3);                  // output pin set
      OCR3AH= 0;
      OCR3AL= brightness;
      break;
    case PIN_6:
      /* timer4, channel A (pin6)  */
      TCCR4A |= (1<<COM4A1) | (1<<COM4A0) | (1<<WGM40);
      TCCR4B |= (1<<WGM42) | (1<<CS40);  // no prescaler
      DDRH |= (1<<PH3);                  // output pin set
      OCR4AH= 0;
      OCR4AL= brightness;
      break;
    case PIN_7:
      /* timer4, channel B (pin7)  */
      TCCR4A |= (1<<COM4B1) | (1<<COM4B0) | (1<<WGM40);
      TCCR4B |= (1<<WGM42) | (1<<CS40);  // no prescaler
      DDRH |= (1<<PH4);                  // output pin set
      OCR4BH= 0;
      OCR4BL= brightness;
      break;
    case PIN_8:
      /* timer4, channel C (pin8)  */
      TCCR4A |= (1<<COM4C1) | (1<<COM4C0) | (1<<WGM40);
      TCCR4B |= (1<<WGM42) | (1<<CS40);  // no prescaler
      DDRH |= (1<<PH5);                  // output pin set
      OCR4CH= 0;
      OCR4CL= brightness;
      break;
    case PIN_46:
      /* timer5, channel A (pin46), Fast PWM, non inverted, 8bit */
      TCCR5A |= (1<<COM5A1) | (1<<COM5A0) | (1<<WGM50);
      TCCR5B |= (1<<WGM52) | (1<<CS50);  // no prescaler
      DDRL |= (1<<PL3);                  // output pin set
      OCR5AH= 0;
      OCR5AL= brightness;
      break;
    case PIN_44:
      /* timer5, channel C (pin44), Fast PWM, non inverted, 8bit */
      TCCR5A |= (1<<COM5C1) | (1<<COM5C0) | (1<<WGM50);
      TCCR5B |= (1<<WGM52) | (1<<CS50);  // no prescaler
      DDRL |= (1<<PL5);                  // output pin set
      OCR5CH= 0;
      OCR5CL= brightness;
      break;

    default:
      RspUport(&rsp);
      break;
  }
}

/* if the packet is valid return 1 and allows its processing */
uint8_t PckCheck(Packet *pck)
{
  //PckInit(pck); /*chiamata utile per resettare index, serve solo per la ritrasmissione (echo) del pacchetto*/

  /* Check control bits of size field */
  uint8_t sizeCtl= (pck->buffer[2]) & SIZECTL_MASK;
  uint8_t sizeControl= 0;
  uint8_t temp= pck->buffer[2] >>3;  /* discard 3 LSB to recover size */
	int k;
	for(k=0; k<5; k++){
		sizeControl += temp&1;      /* counting 1 bits among the first 5 bits of size */
		temp>>=1;
	}
  if(sizeCtl != sizeControl) return 0;

  /* Checksum processing */
  uint8_t cmd= pck->buffer[0];
  uint8_t cksum= pck->buffer[1];
  uint8_t sz= pck->buffer[2];

  uint16_t checksum= 0;
	checksum += cmd+sz;
	int i;
	for(i=3; i<MAX_READ; i++){
		checksum += pck->buffer[i];                /* Checksum processing (no carry)  */
	}
	if(checksum>>8) checksum++;                  /* Checksum carry added if present */
  if(cksum != (uint8_t)checksum) return 0;     /* truncate 1 byte, Little endian system (ATMega 2560)*/
  /* All checks passed */
  RspAck(&rsp);
  return 1;
}

/* packet processing based on command */
void PckProcess(Packet *pck)
{
  uint8_t cmd= pck->buffer[0];
  switch(cmd)
  {
    case 1:
      // handle start()
      break;
    case 2:
      // handle assign dev to port
      break;
    case 3:
      // handle readings from a temp_sensor dev
      break;
    case 4:
      // handle siren setup
      pwmInit(pck->buffer[3],pck->buffer[4]);
      break;
    case 5:
      // handle led setup
      break;

    default:
      RspUcmd(&rsp);
      break;
  }

}

/* write to buffer routine */
uint8_t PckReceive(Packet *pck, uint8_t u8data)
{
  if (pck->index<MAX_READ)
  {
    pck->buffer[pck->index] = u8data;
    pck->index++;
    return 0;
  }
  else return 1;
}

/* read from buffer routine */
uint8_t RspSend(Packet *pck, volatile uint8_t *u8data)
{
  if(pck->index<MAX_WRITE) //inserire MAX_READ per modalità echo, altrimenti MAX_WRITE
  {
    *u8data=pck->buffer[pck->index];
    pck->index++;
    return 0;
  }
  else return 1;
}

/*----------------------------Interrupt driven USART0 routines---------------------------------*/
void USART0Init(void)
{
  /* Set baud rate */
  UBRR0H = (uint8_t)(UBRR_VALUE>>8);
  UBRR0L = (uint8_t)UBRR_VALUE;
  /* Set frame format to 8 data bits, no parity, 1 stop bit */
  UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
  /* Enable reception and RC complete interrupt */
  UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
}

/* RX Complete interrupt service routine */
ISR(USART0_RX_vect)
{
  uint8_t u8temp;
  u8temp=UDR0;
  if ((PckReceive(&pck, u8temp)==1)||(pck.index==MAX_READ))    /* check if packet is complete(index=32) */
  {
    /* disable reception and RX Complete interrupt */
    UCSR0B &= ~((1<<RXEN0)|(1<<RXCIE0));
    /* when reception packed is checked and processed */
    //PckInit(&pck); /* Decommentare per modalità echo */
    if(PckCheck(&pck) != 0)
      PckProcess(&pck);
    /* enable transmission and UDR0 empty interrupt */
    UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);
  }
}

/* UDR0 Empty interrupt service routine */
ISR(USART0_UDRE_vect)
{
  if (RspSend(&rsp, &UDR0)==1||(rsp.index==MAX_WRITE)) //inserire &pck invece di toRead per modalità echo
  {
    /* disable transmission and UDR0 empty interrupt */
    UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
    /* when trasmission is complete index is set back to zero (buffer seen as empty) */
    PckInit(&pck);
    RspInit(&rsp);
    /* enable reception and RC complete interrupt */
    UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
  }
}
/*---------------------------------------------------------------------------------------------*/

/*---------------------------------------> [NOT IN USE]
ISR(TIMER5_COMPA_vect)
{
  if(blinker == 0) // all LED off
  {
    OCR3BL= ZERO;
    OCR3CL= ZERO;
    OCR3AL= ZERO;
    OCR4AL= ZERO;
    OCR4BL= ZERO;
    OCR4CL= ZERO;
    OCR5AL= ZERO;
    OCR5CL= ZERO;
    blinker= 1;
  }
  else            // all LED on
  {
    OCR3BL= *(brightnessLED);
    OCR3CL= *(brightnessLED+1);
    OCR3AL= *(brightnessLED+2);
    OCR4AL= *(brightnessLED+3);
    OCR4BL= *(brightnessLED+4);
    OCR4CL= *(brightnessLED+5);
    OCR5AL= *(brightnessLED+6);
    OCR5CL= *(brightnessLED+7);
    blinker= 0;
  }
}
*/

ISR(PCINT0_vect)
{
  current_pins= (PINB & SWITCH_MASK1);

  /* 255= led off(duty cycle=0%), 0= led on max brightness(duty cycle=100%) */
  /*---------------------------pin7 buttons---------------------------------*/
  // increase button
  if( (current_pins&(1<<PB4)) == 0 )
  {
    if((*(brightnessLED+PIN_7)) > BUTTON_STEP)
    {
      *(brightnessLED+PIN_7)-= BUTTON_STEP;
      OCR4BL= *(brightnessLED+PIN_7);               // use without timer
    }
    else if((*(brightnessLED+PIN_7)) >0)
    {
      *(brightnessLED+PIN_7)= 0;
      OCR4BL= *(brightnessLED+PIN_7);               // use without timer
    }
  }
  // decrease button
  if( (current_pins&(1<<PB5)) == 0 )
  {
    if((*(brightnessLED+PIN_7)) < 255-BUTTON_STEP)
    {
      *(brightnessLED+PIN_7)+= BUTTON_STEP;
      OCR4BL= *(brightnessLED+PIN_7);               // use without timer
    }
    else if((*(brightnessLED+PIN_7)) <255)
    {
      *(brightnessLED+PIN_7)= 255;
      OCR4BL= *(brightnessLED+PIN_7);               // use without timer
    }
  }

  /*---------------------------pin8 buttons---------------------------------*/
  // increase button
  if( (current_pins&(1<<PB6)) == 0 )
  {
    if((*(brightnessLED+PIN_8)) > BUTTON_STEP)
    {
      *(brightnessLED+PIN_8)-= BUTTON_STEP;
      OCR4CL= *(brightnessLED+PIN_8);               // use without timer
    }
    else if((*(brightnessLED+PIN_8)) >0)
    {
      *(brightnessLED+PIN_8)= 0;
      OCR4CL= *(brightnessLED+PIN_8);               // use without timer
    }
  }
  // decrease button
  if( (current_pins&(1<<PB7)) == 0 )
  {
    if((*(brightnessLED+PIN_8)) < 255-BUTTON_STEP)
    {
      *(brightnessLED+PIN_8)+= BUTTON_STEP;
      OCR4CL= *(brightnessLED+PIN_8);               // use without timer
    }
    else if((*(brightnessLED+PIN_8)) <255)
    {
      *(brightnessLED+PIN_8)= 255;
      OCR4CL= *(brightnessLED+PIN_8);               // use without timer
    }
  }

  /*----------------------pin46 buttons----------------------------------*/
  // increase button

  if( (current_pins&(1<<PB0)) == 0 )
  {
    if((*(brightnessLED+PIN_46)) > BUTTON_STEP)
    {
      *(brightnessLED+PIN_46)-= BUTTON_STEP;
      OCR5AL= *(brightnessLED+PIN_46);               // use without timer
    }
    else if((*(brightnessLED+PIN_46)) >0)
    {
      *(brightnessLED+PIN_46)= 0;
      OCR5AL= *(brightnessLED+PIN_46);               // use without timer
    }
  }
  //decrease button
  if( (current_pins&(1<<PB1)) == 0 )
  {
    if((*(brightnessLED+PIN_46)) < 255-BUTTON_STEP)
    {
      *(brightnessLED+PIN_46)+= BUTTON_STEP;
      OCR5AL= *(brightnessLED+PIN_46);               // use without timer
    }
    else if((*(brightnessLED+PIN_46)) <255)
    {
      *(brightnessLED+PIN_46)= 255;
      OCR5AL= *(brightnessLED+PIN_46);               // use without timer
    }
  }

  /*---------------------------pin44 buttons---------------------------------*/
  // increase button
  if( (current_pins&(1<<PB2)) == 0 )
  {
    if((*(brightnessLED+PIN_44)) > BUTTON_STEP)
    {
      *(brightnessLED+PIN_44)-= BUTTON_STEP;
      OCR5CL= *(brightnessLED+PIN_44);               // use without timer
    }
    else if((*(brightnessLED+PIN_44)) >0)
    {
      *(brightnessLED+PIN_44)= 0;
      OCR5CL= *(brightnessLED+PIN_44);               // use without timer
    }
  }
  // decrease button
  if( (current_pins&(1<<PB3)) == 0 )
  {
    if((*(brightnessLED+PIN_44)) < 255-BUTTON_STEP)
    {
      *(brightnessLED+PIN_44)+= BUTTON_STEP;
      OCR5CL= *(brightnessLED+PIN_44);               // use without timer
    }
    else if((*(brightnessLED+PIN_44)) <255)
    {
      *(brightnessLED+PIN_44)= 255;
      OCR5CL= *(brightnessLED+PIN_44);               // use without timer
    }
  }

  _delay_ms(200);                              // help but do not solve bounce contact effect
}

void switchesInit(void)
{
  DDRB &= ~SWITCH_MASK1;      // set SWITCH_MASK1 pins as input
  PORTB |= SWITCH_MASK1;      // enable pull up resistors
  PCICR |= (1 << PCIE0);        // PCINT7:0 pins can cause an interupt
  PCMSK0 |= SWITCH_MASK1;     // enabled only pins 0-4
}

/*----------------------------------------------> [NOT IN USE]
void timerInit(void)
{
  // Timer5, CTC, prescaler=1024
  TCCR5A = 0;
  TCCR5B = (1 << WGM52) | (1 << CS52) | (1 << CS50);
  // 1 ms will correspond do 15.62 counts
  uint16_t ocrval=(uint16_t)(15.62*timer_duration_ms);
  OCR5A = ocrval;
  TIMSK5 |= (1 << OCIE5A);  // enable the timer interrupt
}
*/

int main (void)
{
  PckInit(&pck);        /* Received Packet initialization */
  RspInit(&rsp);  /* Response packet init */
  set_sleep_mode(SLEEP_MODE_IDLE);

  USART0Init();       /* USART0 initialization */
  //timerInit();   ---> [NOT IN USE] <---
  switchesInit();
  sei();              /* Enable global interrupts */
  while(1)
  {
    sleep_mode();  /* Put the device to sleep */
  }
}
