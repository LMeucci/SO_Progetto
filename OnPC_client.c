/*====================================================================================================*/
        /* Serial Port Programming in C (Serial Port Write)                                           */
	      /* Non Cannonical mode                                                                        */
/*----------------------------------------------------------------------------------------------------*/
        /* Program writes a character to the serial port at 19200 bps 8N1 format                      */
	      /* Baudrate - 19200                                                                           */
	      /* Stop bits - 1                                                                              */
	      /* No Parity                                                                                  */
/*----------------------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */

#define MAX_BUF 32

void main(void){

	int fd;            /*File Descriptor*/

	printf("\n +----------------------------------+");
	printf("\n |      SmartHouse: Welcome         |");
	printf("\n +----------------------------------+");

/*------------------------------- Opening the Serial Port -------------------------------*/

  fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY | O_NDELAY); //
      /* /dev/ttyACM0 is the virtual file for the Arduino Board   */
			/* O_RDWR Read/Write access to serial port           */
			/* O_NOCTTY - No terminal will control the process   */
			/* O_NDELAY -Non Blocking Mode,Does not care about-  */
			/* -the status of DCD line,Open() returns immediatly */

	if(fd == -1)						/* Error Checking */
    printf("\n  Error! in Opening ttyACM0  ");
  else
    printf("\n  ttyACM0 Opened Successfully ");


/*---------- Setting the Attributes of the serial port using termios structure --------- */

  struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	cfsetispeed(&SerialPortSettings,B19200); /* Set Read  Speed as 19200                       */
	cfsetospeed(&SerialPortSettings,B19200); /* Set Write Speed as 19200                       */

	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	   /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */


	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode */
	SerialPortSettings.c_oflag &= ~OPOST; /* No Output Processing */

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
    printf("\n  ERROR ! in Setting attributes");
	else
    printf("\n  BaudRate = 19200 \n  StopBits = 1 \n  Parity   = none\n");

/*------------------------------- Write data to serial port -----------------------------*/

	/*             Buffer and packet fields initialization	                       */
	char write_buffer[MAX_BUF]= {0};
	char command= 'A';
	printf("\n  HEX: %x\n",command);
	char packetNumber= 0;
	command |= packetNumber;     /* first 2 command bits are used to store packet number (bit stealing) */

	char size= 6;
	printf("  size: %x\n",size);
	char sizeControl= 0;

	/* size bits only checksum added to size field(only 5 bits needed) with bit stealing */
	char temp= size;
	int k;
	for(k=0; k<5; k++){
		sizeControl += temp&1;      /* counting 1 bits among the first 5 bits of size */
		temp>>=1;
	}
	printf("  sizeControl: %x\n",sizeControl);
	size <<= 3;                  /* making room for size control bits */
	size |= sizeControl;

	char checksum= 0;
	char message[6]= {0};

	/* Assembling the packet */
	*(write_buffer+0)= command;             /* 1° Byte Command     */
	*(write_buffer+1)= checksum;            /* 2° Byte Checksum    */
	*(write_buffer+2)= size;                /* 3° Byte Size        */

	printf("  # packet: %x\n",packetNumber);
	printf("  Calcolo checksum (no carry): ");
	short csum= 0;
	csum += command+size;
	int i;
	for(i=0; i<strlen(message); i++){
		*(write_buffer+i+3)= *(message+i);    /* 4°-32° Byte Payload */
		csum += *(message+i);                 /* Checksum processing (no carry)  */
	}
	printf("%x\n",csum);
	if(csum>>8) csum++;                     /* Checksum carry added if present */
	printf("  Calcolo checksum (carry): %x\n",(char)csum);
	*(write_buffer+1)= (char)csum;          /* Checksum stored in the packet   */


	/*-----------------------------Packet sending-------------------------------------------------------*/
	int  bytes_written  = 0;  	/* Value for storing the number of bytes written to the port            */
	bytes_written = write(fd,write_buffer, MAX_BUF);   /*     use write() to send data to port          */
						           /* "fd"                   - file descriptor pointing to the opened serial port */
									     /*	"write_buffer"         - address of the buffer containing data	            */
									     /* "MAX_BUF" - No of bytes to write                               */
	printf("\n  %s written to ttyACM0",write_buffer);
	printf("\n  %d Bytes written to ttyACM0", bytes_written);
	printf("\n +----------------------------------+\n\n");

	close(fd);/* Close the Serial port */

}
