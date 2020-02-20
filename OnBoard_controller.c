#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define USART_BAUDRATE 19200
#define UBRR_VALUE (F_CPU/16/USART_BAUDRATE-1)

#define MAX_READ 32
#define MAX_WRITE 2
#define SIZECTL_MASK 0x07
#define TO_INT_MASK 0x30
#define ZERO 255

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

const uint16_t timer_duration_ms= 500;
uint8_t brightnessLED[8]={ZERO,ZERO,ZERO,ZERO,ZERO,ZERO,ZERO,ZERO};
volatile uint8_t blinker= 0;

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
                         4=pin7, 5=pin8, 6=pin11, 7=pin12*/
  *(brightnessLED+(pin-TO_INT_MASK))= brightness;
  switch(pin-TO_INT_MASK)
  {
    case 0:
      /* timer3, channel B (pin2) Fast PWM, non inverted, 8bit */
      TCCR3A |= (1<<COM3B1) | (1<<COM3B0) | (1<<WGM30);
      TCCR3B |= (1<<WGM32) | (1<<CS30);  // no prescaler
      DDRE |= (1<<PE4);                  // output pin set
      OCR3BH= 0;
      OCR3BL= brightness;
      break;
    case 1:
      /* timer3, channel C (pin3) Fast PWM, non inverted, 8bit */
      TCCR3A |= (1<<COM3C1) | (1<<COM3C0) | (1<<WGM30);
      TCCR3B |= (1<<WGM32) | (1<<CS30);  // no prescaler
      DDRE |= (1<<PE5);                  // output pin set
      OCR3CH= 0;
      OCR3CL= brightness;
      break;
    case 2:
      /* timer3, channel A (pin5) Fast PWM, non inverted, 8bit */
      TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM30);
      TCCR3B |= (1<<WGM32) | (1<<CS30);  // no prescaler
      DDRE |= (1<<PE3);                  // output pin set
      OCR3AH= 0;
      OCR3AL= brightness;
      break;
    case 3:
      /* timer4, channel A (pin6)  */
      TCCR4A |= (1<<COM4A1) | (1<<COM4A0) | (1<<WGM40);
      TCCR4B |= (1<<WGM42) | (1<<CS40);  // no prescaler
      DDRH |= (1<<PH3);                  // output pin set
      OCR4AH= 0;
      OCR4AL= brightness;
      break;
    case 4:
      /* timer4, channel B (pin7)  */
      TCCR4A |= (1<<COM4B1) | (1<<COM4B0) | (1<<WGM40);
      TCCR4B |= (1<<WGM42) | (1<<CS40);  // no prescaler
      DDRH |= (1<<PH4);                  // output pin set
      OCR4BH= 0;
      OCR4BL= brightness;
      break;
    case 5:
      /* timer4, channel C (pin8)  */
      TCCR4A |= (1<<COM4C1) | (1<<COM4C0) | (1<<WGM40);
      TCCR4B |= (1<<WGM42) | (1<<CS40);  // no prescaler
      DDRH |= (1<<PH5);                  // output pin set
      OCR4CH= 0;
      OCR4CL= brightness;
      break;
    case 6:
      /* timer1, channel A (pin11), Fast PWM, non inverted, 8bit */
      TCCR1A |= (1<<COM1A1) | (1<<COM1A0) | (1<<WGM10);
      TCCR1B |= (1<<WGM12) | (1<<CS10);  // no prescaler
      DDRB |= (1<<PB5);                  // output pin set
      OCR1AH= 0;
      OCR1AL= brightness;
      break;
    case 7:
      /* timer1, channel B (pin12), Fast PWM, non inverted, 8bit */             // NOW pin13 DEBUGGING
      TCCR1A |= (1<<COM1C1) | (1<<COM1C0) | (1<<WGM10);
      TCCR1B |= (1<<WGM12) | (1<<CS10);  // no prescaler
      DDRB |= (1<<PB7);                  // output pin set
      OCR1CH= 0;
      OCR1CL= brightness;
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

ISR(TIMER5_COMPA_vect)
{
  if(blinker == 0) /* all LED off */
  {
    OCR3BL= ZERO;
    OCR3CL= ZERO;
    OCR3AL= ZERO;
    OCR4AL= ZERO;
    OCR4BL= ZERO;
    OCR4CL= ZERO;
    OCR1AL= ZERO;
    OCR1CL= ZERO;                                                               // pin13 DEBUGGING
    blinker= 1;
  }
  else            /* all LED on */
  {
    OCR3BL= *(brightnessLED);
    OCR3CL= *(brightnessLED+1);
    OCR3AL= *(brightnessLED+2);
    OCR4AL= *(brightnessLED+3);
    OCR4BL= *(brightnessLED+4);
    OCR4CL= *(brightnessLED+5);
    OCR1AL= *(brightnessLED+6);
    OCR1CL= *(brightnessLED+7);                                                 // pin13 DEBUGGING
    blinker= 0;
  }
}

void timerInit(void)
{
  /* Timer5, CTC, prescaler=1024 */
  TCCR5A = 0;
  TCCR5B = (1 << WGM52) | (1 << CS52) | (1 << CS50);
  /* 1 ms will correspond do 15.62 counts */
  uint16_t ocrval=(uint16_t)(15.62*timer_duration_ms);
  OCR5A = ocrval;
  TIMSK5 |= (1 << OCIE5A);  // enable the timer interrupt
}

int main (void)
{
  PckInit(&pck);        /* Received Packet initialization */
  RspInit(&rsp);  /* Response packet init */
  set_sleep_mode(SLEEP_MODE_IDLE);

  cli();
  USART0Init();       /* USART0 initialization */
  timerInit();
  sei();              /* Enable global interrupts */
  while(1)
  {
    sleep_mode();  /* Put the device to sleep */
  }
}
