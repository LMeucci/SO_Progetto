#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define USART_BAUDRATE 19200
#define UBRR_VALUE (F_CPU/16/USART_BAUDRATE-1)

#define MAX_SERIAL 8
#define MAX_DEVICES 8 // per type

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
#define SWITCH_MASK 0xff
#define BUTTON_STEP 26

#define CMD_START 0x01
#define CMD_LED_SETUP 0x02
#define CMD_PR_SETUP 0x03
#define CMD_PR_READING 0x04
#define CMD_REMOVE_LED 0x05
#define CMD_REMOVE_PR 0x06

#define CMD_START_SIZE 0x2a
#define CMD_ADC_SIZE 0x02

#define RSP_ACK 0x41
#define RSP_NACK 0x4E
#define RSP_UCMD 0x55   // Uknown command response
#define RSP_UPORT 0x50  // Uknown port response

/*--------------------------------All packet related things----------------------------------*/
typedef struct
{
  uint8_t buffer[MAX_SERIAL];   /* Payload buffer */
  uint8_t index;              /* index to navigate buffer, points to the next empty location */
}Packet;

/* Packet received */
Packet pck;
/* Packet transmitted (response) */
Packet rsp;

/* variables used to handle pwm and timer5      ----> [NOT IN USE] <-----
const uint16_t timer_duration_ms= 500;*/
//volatile uint8_t blinker= 0;

/* pins used to handle switches */
volatile uint8_t current_pins;

/* Packets initialization(erasing) */
void packetBufferInit(Packet *pck)
{
  pck->index=0;    /* starting empty buffer location */
  int i;
  for(i=0; i< MAX_SERIAL; i++)
    pck->buffer[i]= 0;
}

void responseInit(Packet *rsp, uint8_t signal)
{
  rsp->index= 0;
  rsp->buffer[0]= signal;  // response signal stored in the first byte
  rsp->buffer[1]= signal;  // second byte contains checksum
  int  i;
  for(i=2; i< MAX_SERIAL; i++)
    rsp->buffer[i]= 0;
}

/* setup pwm using timer1 */
void pwmInit(uint8_t pin, uint8_t brightness)
{
  /*---------pin mapping:-------------
                         0=pin2, 1=pin3, 2=pin5,  3=pin6,
                         4=pin7, 5=pin8, 6=pin46, 7=pin44*/
  brightness= 255 -brightness; // need to invert user input because brightness=255 -> led off
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
      TCCR5A |= (1<<COM5A1) | (1<<COM5A0) | (1<<WGM50); //
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
      responseInit(&rsp,RSP_UPORT);
      break;
  }
}

/* if the packet is valid return 1 and allows its processing */
uint8_t PckCheck(Packet *pck)
{
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
	for(i=3; i<MAX_SERIAL; i++){
		checksum += pck->buffer[i];                /* Checksum processing (no carry)  */
	}
	if(checksum>>8) checksum++;                  /* Checksum carry added if present */
  if(cksum != (uint8_t)checksum) return 0;     /* truncate 1 byte, Little endian system (ATMega 2560)*/
  /* All checks passed */
  responseInit(&rsp, RSP_ACK);
  return 1;
}

void start(Packet *rsp)
{
  /* preprocessed checksum value(start packet is always the same) cmd+"hello" */
  rsp->buffer[1]= 0xe0;
  /* handshake size + message: HELLO */
  rsp->buffer[2]= CMD_START_SIZE;
  rsp->buffer[3]= CMD_START;
}

void adc_read(uint8_t channel, Packet *rsp)
{
  // select the corresponding channel 0~7
  ADMUX = (ADMUX & 0xF8)|channel; // clears the bottom 3 bits before ORing

  // start single convertion
  // write ’1′ to ADSC
  ADCSRA |= (1<<ADSC);

  // wait for conversion to complete
  // ADSC becomes ’0′ again
  // till then, run loop continuously
  while(ADCSRA & (1<<ADSC));

  rsp->buffer[2]= CMD_ADC_SIZE;
  rsp->buffer[3]= ADCL;
  rsp->buffer[4]= ADCH;
}

void photoresistorInit(uint8_t pin)
{
  /*---------pin mapping:-------------
                         0=pin2, 1=pin3, 2=pin5,  3=pin6,
                         4=pin7, 5=pin8, 6=pin46, 7=pin44 */
  switch(pin-TO_INT_MASK)
  {
    case PIN_2:
      DDRE &= ~(1<<PE4);
      break;
    case PIN_3:
      DDRE &= ~(1<<PE5);
      break;
    case PIN_5:
      DDRE &= ~(1<<PE3);
      break;
    case PIN_6:
      DDRH &= ~(1<<PH3);
      break;
    case PIN_7:
      DDRH &= ~(1<<PH4);
      break;
    case PIN_8:
      DDRH &= ~(1<<PH5);
      break;
    case PIN_46:
      // AREF = AVcc (5V), ADC6 selected
      ADMUX = (1<<REFS0)|(1<<MUX1)|(1<<MUX0);
      // ADC Enable and prescaler of 128, 16000000/128 = 125000 Hz
      ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
      adc_read(PIN_46, &rsp);
      break;
    case PIN_44:
      // AREF = AVcc (5V), ADC7 selected
      ADMUX = (1<<REFS0)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0);
      // ADC Enable and prescaler of 128, 16000000/128 = 125000 Hz
      ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
      break;

    default:
      responseInit(&rsp,RSP_UPORT);
      break;
  }

  //timerInit(); // start timer to get timed readings from temp sensor
}

/* packet processing based on command */
void PckProcess(Packet *pck)
{
  uint8_t cmd= pck->buffer[0];
  switch(cmd)
  {
    case 1:
      // handle start()
      start(&rsp);
      break;
    case 2:
      // handle led setup
      break;
    case 3:
      // handle photoresistor setup
      break;
    case 4:
      // handle photoresistor reading
      pwmInit(pck->buffer[3],pck->buffer[4]);
      break;
    case 5:
      // handle remove led
      photoresistorInit(pck->buffer[3]);
      break;
    case 6:
      //handle remove photoresistor
      break;

    default:
      responseInit(&rsp, RSP_UCMD);
      break;
  }

}

/* write to buffer routine */
uint8_t PckReceive(Packet *pck, uint8_t u8data)
{
  if (pck->index<MAX_SERIAL)
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
  if(pck->index<MAX_SERIAL)
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
  if ((PckReceive(&pck, u8temp)==1)||(pck.index==MAX_SERIAL))    /* check if packet is complete */
  {
    /* disable reception and RX Complete interrupt */
    UCSR0B &= ~((1<<RXEN0)|(1<<RXCIE0));
    /* when reception packet is checked and processed */
    responseInit(&rsp, RSP_NACK);
    if(PckCheck(&pck) != 0)
      PckProcess(&pck);
    /* enable transmission and UDR0 empty interrupt */
    UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);
  }
}

/* UDR0 Empty interrupt service routine */
ISR(USART0_UDRE_vect)
{
  if (RspSend(&rsp, &UDR0)==1||(rsp.index==MAX_SERIAL))
  {
    /* disable transmission and UDR0 empty interrupt */
    UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
    /* when trasmission is complete index is set back to zero (buffer seen as empty) */
    packetBufferInit(&pck);
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

    blinker= 1;
  }
  else            // all LED on
  {

    blinker= 0;
  }
}
*/

ISR(PCINT0_vect)
{
  current_pins= (PINB & SWITCH_MASK);

  /* 255= led off(duty cycle=0%), 0= led on max brightness(duty cycle=100%) */
  /*---------------------------pin7 buttons---------------------------------*/
  // increase button
  if( (current_pins&(1<<PB4)) == 0 )
  {
    if(OCR4BL > BUTTON_STEP)
      OCR4BL-= BUTTON_STEP;
    else if(OCR4BL >0)
      OCR4BL= 0;
  }
  // decrease button
  if( (current_pins&(1<<PB5)) == 0 )
  {
    if(OCR4BL < 255-BUTTON_STEP)
      OCR4BL+= BUTTON_STEP;
    else if(OCR4BL <255)
      OCR4BL= 255;
  }
  /*---------------------------pin8 buttons---------------------------------*/
  // increase button
  if( (current_pins&(1<<PB6)) == 0 )
  {
    if(OCR4CL > BUTTON_STEP)
      OCR4CL-= BUTTON_STEP;
    else if(OCR4CL >0)
      OCR4CL= 0;
  }
  // decrease button
  if( (current_pins&(1<<PB7)) == 0 )
  {
    if(OCR4CL < 255-BUTTON_STEP)
      OCR4CL+= BUTTON_STEP;
    else if(OCR4CL <255)
      OCR4CL= 255;
  }
  /*----------------------pin46 buttons----------------------------------*/
  // increase button
  if( (current_pins&(1<<PB0)) == 0 )
  {
    if(OCR5AL > BUTTON_STEP)
      OCR5AL-= BUTTON_STEP;
    else if(OCR5AL >0)
      OCR5AL= 0;
  }
  //decrease button
  if( (current_pins&(1<<PB1)) == 0 )
  {
    if(OCR5AL < 255-BUTTON_STEP)
      OCR5AL += BUTTON_STEP;

    else if(OCR5AL <255)
      OCR5AL= 255;
  }
  /*---------------------------pin44 buttons---------------------------------*/
  // increase button
  if( (current_pins&(1<<PB2)) == 0 )
  {
    if(OCR5CL > BUTTON_STEP)
      OCR5CL-= BUTTON_STEP;
    else if(OCR5CL >0)
      OCR5CL= 0;
  }
  // decrease button
  if( (current_pins&(1<<PB3)) == 0 )
  {
    if(OCR5CL < 255-BUTTON_STEP)
      OCR5CL+= BUTTON_STEP;
    else if(OCR5CL <255)
      OCR5CL= 255;
  }
  _delay_ms(200);                              // help but do not solve bounce contact effect
}

void switchesInit(void)
{
  DDRB &= ~SWITCH_MASK;      // set SWITCH_MASK pins as input
  PORTB |= SWITCH_MASK;      // enable pull up resistors
  PCICR |= (1 << PCIE0);        // PCINT7:0 pins can cause an interupt
  PCMSK0 |= SWITCH_MASK;     // enabled only pins 0-4
}
/*
void timerInit(void)
{
  // Timer2, CTC, prescaler=1024
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
  packetBufferInit(&pck);        /* Received Packet initialization */
  responseInit(&rsp, RSP_NACK);  /* Response packet init */
  set_sleep_mode(SLEEP_MODE_IDLE);

  USART0Init();       /* USART0 initialization */
  switchesInit();
  sei();              /* Enable global interrupts */
  while(1)
  {
    sleep_mode();  /* Put the device to sleep */
  }
}
