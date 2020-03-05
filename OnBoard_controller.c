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
#define PIN_11 6
#define PIN_12 7
#define PIN_ADC_DEFAULT 8 // pin 13 on Arduino mega

#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3
#define ADC4 4
#define ADC5 5
#define ADC6 6
#define ADC7 7

#define SIZECTL_MASK 0x07
#define CHAR_TO_HEX 0x30
#define SWITCH_MASK 0xff
#define BUTTON_STEP 26
#define OP_ON 0x4F
#define OP_OFF 0x46
#define TIMER_QUARTER_MINUTE 1500  // 10 ms timer x 1500= 15sec
#define PIN_ADC_DEFAULT_MASK 0x80

#define CMD_START 0x01
#define CMD_RESET 0x02
#define CMD_LED_SETUP 0x03
#define CMD_PR_SETUP 0x04
#define CMD_PR_READING 0x05
#define CMD_REMOVE_LED 0x06
#define CMD_REMOVE_PR 0x07

#define CMD_START_SIZE 0x05
#define CMD_RESET_SIZE 0x00
#define CMD_LED_SETUP_SIZE 0x02
#define CMD_PR_SETUP_SIZE 0x02
#define CMD_PR_READING_SIZE 0x01
#define CMD_REMOVE_LED_SIZE 0x01
#define CMD_REMOVE_PR_SIZE 0x01

#define RSP_ACK 0x41
#define RSP_NACK 0x4E

/*--------------------------------All packet related things----------------------------------*/
typedef struct
{
  uint8_t buffer[MAX_SERIAL];   /* contains data received/to send */
  uint8_t index;                /* index to navigate buffer, points to the next empty location */
}Packet;

/* Packet received */
Packet pck;
/* Packet transmitted (response) */
Packet rsp;

// timer2 variables (used in adc reading)
const uint16_t timer_duration_ms= 10;
volatile uint16_t timerAuxCompare= 0;

/* pins used to handle switches */
volatile uint8_t current_pins;

// keeps track of pwm pins in use, used to implement ledRemove
uint8_t pinsPWM= 0;
// keep track of adc pins in use and led ports associated to them
uint8_t pinsADC= 0;
uint8_t pinsADCtoLED[MAX_DEVICES]= {0};
// keep track of adc readings, used from photoresistorReading to send stored values
volatile uint16_t adcReadings[MAX_DEVICES]= {0};

/*-----------------------------Initialization functions----------------------------------- */
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
  int  i;
  for(i=1; i< MAX_SERIAL; i++)
    rsp->buffer[i]= 0;
}

void timerADCInit(void)
{
  // Timer2, CTC, prescaler=1024
  TCCR2A = 0;
  TCCR2B = (1 << WGM22) | (1 << CS22) | (1 << CS20);
  // 1 ms will correspond do 15.62 counts
  uint8_t ocrval=(uint8_t)(15.62*timer_duration_ms);
  OCR2A = ocrval;
  cli();
  TIMSK2 |= (1 << OCIE2A);  // enable the timer output compare match A interrupt
  sei();
}

void switchesInit(void)
{
  DDRK &= ~SWITCH_MASK;      // set SWITCH_MASK pins as input
  PORTK |= SWITCH_MASK;      // enable pull up resistors
  cli();
  PCICR |= (1 << PCIE2);     // PCINT23:16 pins can cause an interupt
  PCMSK2 |= SWITCH_MASK;     // enabled all 8 pins
  sei();
}

void adcDefaultPinInit(void)
{
  DDRB |= PIN_ADC_DEFAULT_MASK; // pin 13 (led pin) set as output
}

/*-----------------------------Board functionalities---------------------------------------*/
void start(Packet *pck, Packet *rsp)
{
  /* check handshake */
  if( pck->buffer[3] != 0x48   // H
   || pck->buffer[4] != 0x45   // E
   || pck->buffer[5] != 0x4C   // L
   || pck->buffer[6] != 0x4C   // L
   || pck->buffer[7] != 0x4F ) // O
   {
     rsp->buffer[0] = RSP_NACK;
   }
  else // handshake successful
  {
    /* start response packet */
    rsp->buffer[2]= CMD_START;
    rsp->buffer[3]= CMD_START_SIZE;
  }
}

void reset(Packet *rsp)
{
  // disabling all pwm pins
  pinsPWM= 0;

  TCCR3A= 0; TCCR3B= 0;
  DDRE &= ~((1<<PE3)|(1<<PE4)|(1<<PE5));
  OCR3A= 0; OCR3B= 0; OCR3C= 0;

  TCCR4A= 0; TCCR4B= 0;
  DDRH &= ~((1<<PH3)|(1<<PH4)|(1<<PH5));
  OCR4A= 0; OCR4B= 0; OCR4C= 0;

  TCCR1A= 0; TCCR1B= 0;
  DDRB &= ~((1<<PB5)|(1<<PB6));
  OCR1A= 0; OCR1B= 0;

  // disabling adc
  pinsADC= 0;
  int i;
  for(i=0; i<MAX_DEVICES; i++)
  {
    pinsADCtoLED[i]= 0;
    adcReadings[i]= 0;
  }

  ADMUX= 0;
  ADCSRA= 0;
  // disable timer2 associated with adc
  cli();
  TIMSK2= 0;
  sei();
  TCCR2B= 0;
  OCR2A= 0;

  /* reset response packet */
  rsp->buffer[2]= CMD_RESET;
  rsp->buffer[3]= CMD_RESET_SIZE;
}

void ledSetup(uint8_t pin, uint8_t brightness, Packet *rsp)
{
  /*---------pin mapping:-------------
                         0=pin2, 1=pin3, 2=pin5,  3=pin6,
                         4=pin7, 5=pin8, 6=pin46, 7=pin44*/

  brightness= 255 -brightness; // need to invert user input because brightness=255 -> led off
  switch(pin)
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
      /* timer4, channel A (pin6) Fast PWM, non inverted, 8bit */
      TCCR4A |= (1<<COM4A1) | (1<<COM4A0) | (1<<WGM40);
      TCCR4B |= (1<<WGM42) | (1<<CS40);  // no prescaler
      DDRH |= (1<<PH3);                  // output pin set
      OCR4AH= 0;
      OCR4AL= brightness;
      break;

    case PIN_7:
      /* timer4, channel B (pin7) Fast PWM, non inverted, 8bit */
      TCCR4A |= (1<<COM4B1) | (1<<COM4B0) | (1<<WGM40);
      TCCR4B |= (1<<WGM42) | (1<<CS40);  // no prescaler
      DDRH |= (1<<PH4);                  // output pin set
      OCR4BH= 0;
      OCR4BL= brightness;
      break;

    case PIN_8:
      /* timer4, channel C (pin8) Fast PWM, non inverted, 8bit */
      TCCR4A |= (1<<COM4C1) | (1<<COM4C0) | (1<<WGM40);
      TCCR4B |= (1<<WGM42) | (1<<CS40);  // no prescaler
      DDRH |= (1<<PH5);                  // output pin set
      OCR4CH= 0;
      OCR4CL= brightness;
      break;

    case PIN_11:
      /* timer1, channel A (pin11), Fast PWM, non inverted, 8bit */
      TCCR1A |= (1<<COM1A1) | (1<<COM1A0) | (1<<WGM10); //
      TCCR1B |= (1<<WGM12) | (1<<CS10);  // no prescaler
      DDRB |= (1<<PB5);                  // output pin set
      OCR1AH= 0;
      OCR1AL= brightness;
      break;

    case PIN_12:
      /* timer1, channel B (pin12), Fast PWM, non inverted, 8bit */
      TCCR1A |= (1<<COM1B1) | (1<<COM1B0) | (1<<WGM10);
      TCCR1B |= (1<<WGM12) | (1<<CS10);  // no prescaler
      DDRB |= (1<<PB6);                  // output pin set
      OCR1BH= 0;
      OCR1BL= brightness;
      break;
  }
  pinsPWM |= (1<<pin);
  rsp->buffer[2]= CMD_LED_SETUP;
  rsp->buffer[3]= CMD_LED_SETUP_SIZE;
}

void photoresistorSetup(uint8_t pin, uint8_t led, Packet *rsp)
{
  if( pinsADC == 0 )
  {
    // AREF = AVcc (5V)
    ADMUX = (1<<REFS0);
    // ADC enable, prescaler=128 16000000/128 = 125000 Hz
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
    timerADCInit();
  }
  pinsADC |= (1<<pin);
  pinsADCtoLED[pin]= led;

  rsp->buffer[2]= CMD_PR_SETUP;
  rsp->buffer[3]= CMD_PR_SETUP_SIZE;
}

void photoresistorReading(uint8_t pin, Packet *rsp)
{
  /* preparing response packet */
  rsp->buffer[2]= CMD_PR_READING;
  rsp->buffer[3]= CMD_PR_READING_SIZE;
  rsp->buffer[4]= (uint8_t) adcReadings[pin];
  rsp->buffer[5]= (uint8_t)(adcReadings[pin]>>8);
}

void ledRemove(uint8_t pin, Packet *rsp)
{
  switch(pin)
  {
    case PIN_2:
      if( (pinsPWM&(0x06)) == 0 ) // pin 3 & 5 are not in use, disable pwm bound to timer 3
        { TCCR3A= 0; TCCR3B= 0; }
      else                        // disable only channel associated with pin
        { TCCR3A &= ~((1<<COM3B1) | (1<<COM3B0)); }
      DDRE &= ~(1<<PE4);  // output pin disabled
      OCR3B= 0;
      break;

    case PIN_3:
      if( (pinsPWM&(0x05)) == 0 ) // pin 2 & 5 are not in use, disable pwm bound to timer 3
        { TCCR3A= 0; TCCR3B= 0; }
      else
        { TCCR3A &= ~((1<<COM3C1) | (1<<COM3C0)); }
      DDRE &= ~(1<<PE5);
      OCR3C= 0;
      break;

    case PIN_5:
      if( (pinsPWM&(0x03)) == 0 ) // pin 2 & 3 are not in use, disable pwm bound to timer 3
        { TCCR3A= 0; TCCR3B= 0; }
      else
        { TCCR3A &= ~((1<<COM3A1) | (1<<COM3A0)); }
      DDRE &= ~(1<<PE3);
      OCR3A= 0;
      break;

    case PIN_6:
      if( (pinsPWM&(0x30)) == 0 ) // pin 7 & 8 are not in use, disable pwm bound to timer 4
        { TCCR4A= 0; TCCR4B= 0; }
      else
        { TCCR4A &= ~((1<<COM4A1) | (1<<COM4A0)); }
      DDRH &= ~(1<<PH3);
      OCR4A= 0;
      break;

    case PIN_7:
      if( (pinsPWM&(0x28)) == 0 ) // pins 6 & 8 are not in use, disable pwm bound to timer 4
        { TCCR4A= 0; TCCR4B= 0; }
      else
      { TCCR4A &= ~((1<<COM4B1) | (1<<COM4B0)); }
      DDRH &= ~(1<<PH4);
      OCR4B= 0;
      break;

    case PIN_8:
      if( (pinsPWM&(0x18)) == 0 ) // pins 6 & 7 are not in use, disable pwm bound to timer 4
        { TCCR4A= 0; TCCR4B= 0; }
      else
      { TCCR4A &= ~((1<<COM4C1) | (1<<COM4C0)); }
      DDRH &= ~(1<<PH5);
      OCR4C= 0;
      break;

    case PIN_11:
      if( (pinsPWM&(0x80)) == 0) // pin 12 is not in use, disable pwm bound to timer 1
        { TCCR1A= 0; TCCR1B= 0; }
      else
      { TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0)); }
      DDRB &= ~(1<<PB5);
      OCR1A= 0;
      break;

    case PIN_12:
      if( (pinsPWM&(0x40)) == 0) // pin 11 is not in use, disable pwm bound to timer 1
        { TCCR1A= 0; TCCR1B= 0; }
      else
      { TCCR1A &= ~((1<<COM1B1) | (1<<COM1B0)); }
      DDRB &= ~(1<<PB6);
      OCR1B= 0;
      break;
  }
  /* removing pin from variable which keeps track of all pins in use */
  pinsPWM &= ~(1<<pin);
  /* preparing response packet */
  rsp->buffer[2]= CMD_REMOVE_LED;
  rsp->buffer[3]= CMD_REMOVE_LED_SIZE;
}

void photoresistorRemove(uint8_t pin, Packet *rsp)
{
  /* removing pin from variable which keeps track of all pins in use */
  pinsADC &= ~(1<<pin);
  pinsADCtoLED[pin]= 0; // removing port associated to pin
  if(pinsADC == 0)
  {
    // disable ADC
    ADMUX= 0;
    ADCSRA= 0;
    // disable timer2
    cli();
    TIMSK2= 0;
    sei();
    TCCR2B= 0;
    OCR2A= 0;
  }
  /* preparing response packet */
  rsp->buffer[2]= CMD_REMOVE_PR;
  rsp->buffer[3]= CMD_REMOVE_PR_SIZE;
}

/*---------------------------Packets Handling (received and response)-----------------------*/

/* if the packet is valid return 0 and allows its processing */
uint8_t pckCheck(Packet *pck)
{
  /* Check if command received is a valid command */
  if( (pck->buffer[0]>CMD_REMOVE_PR) || (pck->buffer[0]<CMD_START) ) return -1;
  /* Check control bits of size field */
  uint8_t sizeCtl= (pck->buffer[2]) & SIZECTL_MASK;
  uint8_t sizeControl= 0;
  uint8_t temp= pck->buffer[2] >>3;  /* discard 3 LSB to recover size */
	int k;
	for(k=0; k<5; k++){
		sizeControl += temp&1;      /* counting 1 bits among the first 5 bits of size */
		temp>>=1;
	}
  if(sizeCtl != sizeControl) return -1;

  /* Checksum processing */
  uint8_t cmd= pck->buffer[0];
  uint8_t cksum= pck->buffer[1];
  uint8_t sz= pck->buffer[2];

  uint16_t checksum= 0;
	checksum += cmd+sz;
	int i;
	for(i=3; i<MAX_SERIAL; i++){
		checksum += pck->buffer[i];             /* Checksum processing (no carry)  */
	}
	if(checksum>>8) checksum++;               /* Checksum carry added if present */
  if(cksum != (uint8_t)checksum) return -1; /* truncate 1 byte, Little endian system (ATMega 2560) */
  /* All checks passed */
  responseInit(&rsp, RSP_ACK);
  return 0;
}

/* packet processing based on command */
void pckProcess(Packet *pck, Packet *rsp)
{
  uint8_t cmd= pck->buffer[0];
  switch(cmd)
  {
    case 1:
      // handle start
      start(pck,rsp);
      break;
    case 2:
      // handle reset
      reset(rsp);
      break;
    case 3:
      // handle led setup
      ledSetup(pck->buffer[3],pck->buffer[4],rsp);
      break;
    case 4:
      // handle photoresistor setup
      photoresistorSetup(pck->buffer[3],pck->buffer[4],rsp);
      break;
    case 5:
      // handle photoresistor reading
      photoresistorReading(pck->buffer[3],rsp);
      break;
    case 6:
      // handle remove led
      ledRemove(pck->buffer[3],rsp);
      break;
    case 7:
      // handle remove photoresistor
      photoresistorRemove(pck->buffer[3],rsp);
      break;
  }

}

/* write to buffer routine */
uint8_t pckReceive(Packet *pck, uint8_t u8data)
{
  if (pck->index<MAX_SERIAL)
  {
    pck->buffer[pck->index] = u8data;
    pck->index++;
    return 0;
  }
  else return 1;
}

void rspChecksum(Packet *rsp)
{
  /* "adding" size-only checksum to size with bitstealing(3bits) */
  uint8_t size= rsp->buffer[3];
  uint8_t sizeControl= 0;
	uint8_t temp= size;
	uint8_t k;
	for(k=0; k<5; k++)
	{
		sizeControl += temp&1;      // counting 1 bits among the first 5 bits of size
		temp>>=1;
	}
	size <<= 3;                  // space for sizeControl bits
	size |= sizeControl;
  uint8_t sizeCopy= rsp->buffer[3]; // saving size for checksum processing
  rsp->buffer[3]= size;

  /* checksum processed and added to the response */
  uint16_t csum= 0;
  // ack+cmd+size
	csum += rsp->buffer[0]+rsp->buffer[2]+rsp->buffer[3];
	uint8_t i;
	for(i=0; i<sizeCopy; i++)
		csum += rsp->buffer[i+4];         // Checksum processing (no carry)

	if(csum>>8) csum++;                 // Checksum carry added if present
  rsp->buffer[1]= (uint8_t)csum;      // Little endian system (ATMega 2560)
}

/* read from buffer routine */
uint8_t rspSend(Packet *pck, volatile uint8_t *u8data)
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
  if ((pckReceive(&pck, u8temp)==1)||(pck.index==MAX_SERIAL))    /* check if packet is complete */
  {
    /* disable reception and RX Complete interrupt */
    UCSR0B &= ~((1<<RXEN0)|(1<<RXCIE0));
    /* when reception packet is checked and processed */
    responseInit(&rsp, RSP_NACK);
    if(pckCheck(&pck) == 0)
      pckProcess(&pck,&rsp);
    rspChecksum(&rsp);
    /* enable transmission and UDR0 empty interrupt */
    UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);
  }
}

/* UDR0 Empty interrupt service routine */
ISR(USART0_UDRE_vect)
{
  if (rspSend(&rsp, &UDR0)==1||(rsp.index==MAX_SERIAL))
  {
    /* disable transmission and UDR0 empty interrupt */
    UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
    /* when trasmission is complete index is set back to zero (buffer seen as empty) */
    packetBufferInit(&pck);
    /* enable reception and RC complete interrupt */
    UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
  }
}

/*----------------------------Other interrupt service routines--------------------------------*/

ISR(TIMER2_COMPA_vect)
{
  /*---------pin mapping:-------------
                         0=pinA0, 1=pinA1, 2=pinA2,  3=pinA3,
                         4=pinA4, 5=pinA5, 6=pinA6, 7=pinA7 */

  // adc read only once every 15sec
  timerAuxCompare++;
  if(timerAuxCompare == TIMER_QUARTER_MINUTE)
  {
    timerAuxCompare= 0;

    int pin;
    for(pin=0; pin<MAX_DEVICES; pin++)
    {
      // check if a photoresistor is setup at pin
      if( ((pinsADC)&(1<<pin)) != 0)
      {
        ADMUX = (ADMUX & 0xF8); // clearing last 3 bits before ORing
        switch(pin)
        {
          case ADC0:
            break;
          case ADC1:
            ADMUX |= (1<<MUX0);
            break;
          case ADC2:
            ADMUX |= (1<<MUX1);
            break;
          case ADC3:
            ADMUX |= (1<<MUX1)|(1<<MUX0);
            break;
          case ADC4:
            ADMUX |= (1<<MUX2);
            break;
          case ADC5:
            ADMUX |= (1<<MUX2)|(1<<MUX0);
            break;
          case ADC6:
            ADMUX |= (1<<MUX2)|(1<<MUX1);
            break;
          case ADC7:
            ADMUX |= (1<<MUX2)|(1<<MUX1)|(1<<MUX0);
            break;
          }
        // start single conversion for selected pin, write 1 to ADSC
        ADCSRA |= (1<<ADSC);
        // wait for conversion to complete
        while(ADCSRA & (1<<ADSC));
        adcReadings[pin]= ADC;
        // ADC range 0-1023, brightness range 0-255, division by 4 adjust result in a trivial way
        int brightnessLevel= (ADC/4);
        // trim brightness according to daylight values registered(light off)
        brightnessLevel= ((brightnessLevel+70)>255)? 255 : brightnessLevel+70;
        switch(pinsADCtoLED[pin])
        {
          case PIN_2:
            OCR3BL= brightnessLevel;
            break;
          case PIN_3:
            OCR3CL= brightnessLevel;
            break;
          case PIN_5:
            OCR3AL= brightnessLevel;
            break;
          case PIN_6:
            OCR4AL= brightnessLevel;
            break;
          case PIN_7:
            OCR4BL= brightnessLevel;
            break;
          case PIN_8:
            OCR4CL= brightnessLevel;
            break;
          case PIN_11:
            OCR1AL= brightnessLevel;
            break;
          case PIN_12:
            OCR1BL= brightnessLevel;
            break;
          case PIN_ADC_DEFAULT:
            if(brightnessLevel < 240) // dayight trigger
              PORTB |= PIN_ADC_DEFAULT_MASK;
            else
              PORTB &= ~(PIN_ADC_DEFAULT_MASK);
            break;
        }
      }
    }
  }
}

ISR(PCINT2_vect)
{
  current_pins= (PINK & SWITCH_MASK);

  /* 255= led off(duty cycle=0%), 0= led on max brightness(duty cycle=100%) */
  /*---------------------------pin7 buttons---------------------------------*/
  // increase button
  if( (current_pins&(1<<PK0)) == 0 )
  {
    if(OCR4BL > BUTTON_STEP)
      OCR4BL-= BUTTON_STEP;
    else if(OCR4BL >0)
      OCR4BL= 0;
  }
  // decrease button
  if( (current_pins&(1<<PK1)) == 0 )
  {
    if(OCR4BL < 255-BUTTON_STEP)
      OCR4BL+= BUTTON_STEP;
    else if(OCR4BL <255)
      OCR4BL= 255;
  }
  /*---------------------------pin8 buttons---------------------------------*/
  // increase button
  if( (current_pins&(1<<PK2)) == 0 )
  {
    if(OCR4CL > BUTTON_STEP)
      OCR4CL-= BUTTON_STEP;
    else if(OCR4CL >0)
      OCR4CL= 0;
  }
  // decrease button
  if( (current_pins&(1<<PK3)) == 0 )
  {
    if(OCR4CL < 255-BUTTON_STEP)
      OCR4CL+= BUTTON_STEP;
    else if(OCR4CL <255)
      OCR4CL= 255;
  }
  /*----------------------pin11 buttons----------------------------------*/
  // increase button
  if( (current_pins&(1<<PK4)) == 0 )
  {
    if(OCR1AL > BUTTON_STEP)
      OCR1AL-= BUTTON_STEP;
    else if(OCR1AL >0)
      OCR1AL= 0;
  }
  //decrease button
  if( (current_pins&(1<<PK5)) == 0 )
  {
    if(OCR1AL < 255-BUTTON_STEP)
      OCR1AL += BUTTON_STEP;

    else if(OCR1AL <255)
      OCR1AL= 255;
  }
  /*---------------------------pin12 buttons---------------------------------*/
  // increase button
  if( (current_pins&(1<<PK6)) == 0 )
  {
    if(OCR1BL > BUTTON_STEP)
      OCR1BL-= BUTTON_STEP;
    else if(OCR1BL >0)
      OCR1BL= 0;
  }
  // decrease button
  if( (current_pins&(1<<PK7)) == 0 )
  {
    if(OCR1BL < 255-BUTTON_STEP)
      OCR1BL+= BUTTON_STEP;
    else if(OCR1BL <255)
      OCR1BL= 255;
  }
  _delay_ms(200);            // help with but do not solve bounce contact effect
}


int main (void)
{
  packetBufferInit(&pck);        /* Received Packet initialization */
  responseInit(&rsp, RSP_NACK);  /* Response packet init */
  set_sleep_mode(SLEEP_MODE_IDLE);

  USART0Init();       /* USART0 initialization */
  switchesInit();
  adcDefaultPinInit();
  sei();              /* Enable global interrupts */
  while(1)
  {
    sleep_mode();  /* Put the device to sleep */
  }
}
