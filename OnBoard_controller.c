#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define USART_BAUDRATE 19200
#define UBRR_VALUE (F_CPU/16/USART_BAUDRATE-1)
#define BUF_SIZE 32

/*-----------------------Definition of packet structure & operations---------------------------*/
typedef struct{
  uint8_t buffer[BUF_SIZE];   /* Payload buffer */
  uint8_t index;              /* index to navigate buffer, points to the next empty location */
}Packet;

/* Packet declaration */
Packet pck;

volatile uint8_t packetChunk= 0;       /* To keep track of bytes received (current packet)  */
volatile uint8_t nextPacket= 0;        /* Sequence number of the next packet to be received */


/* Packet initialization(erasing) */
void PacketInit(Packet *pck)
{
  pck->index=0;    /* starting empty buffer location */
}

/* write to buffer routine */
uint8_t PacketWrite(Packet *pck, uint8_t u8data)
{
  if (pck->index<BUF_SIZE)
  {
    pck->buffer[pck->index] = u8data;
    pck->index++;
    return 0;
  }
  else return 1;
}

/* read from buffer routine */
uint8_t PacketRead(Packet *pck, volatile uint8_t *u8data)
{
  if(pck->index<BUF_SIZE)
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
  if ((PacketWrite(&pck, u8temp)==1)||(pck.index==BUF_SIZE))    /* check if packet is complete(index=32) */
  {
    /* when reception is complete index is set back to zero to start trasmission */
    PacketInit(&pck);
    /* disable reception and RX Complete interrupt */
    UCSR0B &= ~((1<<RXEN0)|(1<<RXCIE0));
    /* enable transmission and UDR0 empty interrupt */
    UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);
  }
}

/* UDR0 Empty interrupt service routine */
ISR(USART0_UDRE_vect)
{
  if (PacketRead(&pck, &UDR0)==1)
  {
    /* when trasmission is complete index is set back to zero (buffer seen as empty) */
    PacketInit(&pck);
    /* disable transmission and UDR0 empty interrupt */
    UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
    /* enable reception and RC complete interrupt */
    UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
  }
}
/*---------------------------------------------------------------------------------------------*/


int main (void)
{
  PacketInit(&pck);   /* Packet initialization */
  set_sleep_mode(SLEEP_MODE_IDLE);
  USART0Init();       /* USART0 initialization */
  sei();              /* Enable global interrupts */
  while(1)
  {
    sleep_mode();  /* Put the device to sleep */

    //if(packetComplete) ...
  }
}
