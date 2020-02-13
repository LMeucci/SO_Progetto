#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define USART_BAUDRATE 19200
#define UBRR_VALUE (F_CPU/16/USART_BAUDRATE-1)

#define MAX_READ 32
#define MAX_WRITE 2

#define MAX_SEQNUM 4
#define SEQNUM_MASK 0x03
#define SIZECTL_MASK 0x07

#define ACK 0x41
#define NACK 0x4E
#define UCMD 0x55

/*--------------------------------All packet related things----------------------------------*/
typedef struct{
  uint8_t buffer[MAX_READ];   /* Payload buffer */
  uint8_t index;              /* index to navigate buffer, points to the next empty location */
}Packet;

/* Packet declaration */
Packet pck;
/* Response packets */
Packet rsp;
/*Sequence number of the next packet to be received */
uint8_t nextPacket= 0;

/* Packets initialization(erasing) */
void PckInit(Packet *pck)
{
  pck->index=0;    /* starting empty buffer location */
}

void RspInit(Packet *rsp)
{
  rsp->index=0;
  rsp->buffer[0]= NACK;   /* NAck response stored in the first byte */
  rsp->buffer[1]= NACK;   /* Only one field so checksum is just its copy */
}

void RspAck(Packet *rsp)
{
  rsp->buffer[0]= ACK;   /* Ack response */
  rsp->buffer[1]= ACK;
}

void RspUcmd()
{
  rsp->buffer[0]= UCMD;   /* Unknown command response */
  rsp->buffer[1]= UCMD;
}

/* if the packet is valid return 1 and allows its processing */
uint8_t PckCheck(Packet *pck)
{
  //PckInit(pck); /*chiamata utile per resettare index, serve solo per la ritrasmissione (echo) del pacchetto*/

  /* Check sequence number */
  //uint8_t seqNum= (pck->buffer[0]) & SEQNUM_MASK;
  //if(seqNum != nextPacket) return 0;
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

void PckProcess(Packet *pck)
{
  /* Basandomi sul campo cmd seleziono l'operazione da eseguire(chiamata a funzione)
     tramite switch case */
  /*
  uint8_t cmd= pck->buffer[0];
  switch(cmd)
  {
    case 1:
    case 2:
    case 3:


    default:
      RspUcmd(&rsp);
      break;
  }
  */
  /* setting next packet number (incremented only if a processing occurs). It's the last op */
  nextPacket= (nextPacket+1)%MAX_SEQNUM;  /* packets number cycle from 0 to 3 */
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
    if(PckCheck(&pck))
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


int main (void)
{
  PckInit(&pck);        /* Received Packet initialization */
  RspInit(&rsp);  /* Response packet init */
  set_sleep_mode(SLEEP_MODE_IDLE);
  USART0Init();       /* USART0 initialization */
  sei();              /* Enable global interrupts */
  while(1)
  {
    sleep_mode();  /* Put the device to sleep */

  }
}


/* terminare PckProcess()
  decidere che tipo di pacchetti invia la Board a PC
  sicuri ack e nack



*/
