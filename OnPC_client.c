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

#define MAX_WRITE 32
#define MAX_READ 2

void main(void)
{
	int fd;            /*File Descriptor*/

	printf("\n +----------------------------------+");
	printf("\n |      SmartHouse: Welcome         |");
	printf("\n +----------------------------------+");

/*------------------------------- Opening the Serial Port -------------------------------*/

  fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY ); //| O_NDELAY
      /* /dev/ttyACM0 is the virtual file for the Arduino Board   */
			/* O_RDWR Read/Write access to serial port           */
			/* O_NOCTTY - No terminal will control the process   */
			/* O_NDELAY -Non Blocking Mode,Does not care about-  */
			/* -the status of DCD line,Open() returns immediatly */

	if(fd == -1)						/* Error Checking */
	{
		#ifdef DEBUG
			printf("\n  Error! in Opening ttyACM0  ");
		#endif
	}
  else
	{
		#ifdef DEBUG
			printf("\n  ttyACM0 Opened Successfully ");
		#endif
		//fcntl(fd, F_SETFL,0);  /* so read() works properly in non canonical mode */
	}

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
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Canonical mode | ECHONL */
																				/*Disable special handling of bytes*/
	SerialPortSettings.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
	SerialPortSettings.c_oflag &= ~(OPOST); /* No Output Processing |ONLCR */

	SerialPortSettings.c_cc[VTIME] = 0; /* Blocking read, unlimited wait */
	SerialPortSettings.c_cc[VMIN] = 2; /* read() waits for 2 bytes before returning */

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
	{
		#ifdef DEBUG
			printf("\n  ERROR ! in Setting attributes");
		#endif
	}
	else
	{
		#ifdef DEBUG
			printf("\n  BaudRate = 19200 \n  StopBits = 1 \n  Parity   = none\n");
		#endif
	}



	tcflush(fd, TCIOFLUSH); // clear buffer

/*------------------------------- Write data to serial port -----------------------------*/

	/*             Buffer and packet fields initialization	                       */
	unsigned char write_buffer[MAX_WRITE]= {0};
	char command= 'Z';
	#ifdef DEBUG
		printf("\n  HEX: %x\n",command);
	#endif
	// disabled functionality still testing
	//char packetNumber= 0;
	//command |= packetNumber;     /* first 2 command bits are used to store packet number (bit stealing) */

	char size= 5;
	#ifdef DEBUG
		printf("  size: %x\n",size);
	#endif

	char sizeControl= 0;
	/* size-bits-only-checksum added to size field(only 5 bits needed) with bit stealing */
	char temp= size;
	int k;
	for(k=0; k<5; k++)
	{
		sizeControl += temp&1;      /* counting 1 bits among the first 5 bits of size */
		temp>>=1;
	}

	#ifdef DEBUG
		printf("  sizeControl: %x\n",sizeControl);
	#endif
	size <<= 3;                  /* making room for size control bits */
	size |= sizeControl;

	unsigned char checksum= 0;
	char message[5]= "testo";

	/* Assembling the packet */
	*(write_buffer+0)= command;             /* 1° Byte Command     */
	*(write_buffer+1)= checksum;            /* 2° Byte Checksum    */
	*(write_buffer+2)= size;                /* 3° Byte Size        */

	/* Checksum processing */
	#ifdef DEBUG
		//printf("  # packet: %x\n",packetNumber); /* Disabled functionality */
		printf("  Calcolo checksum (no carry): ");
	#endif
	unsigned short csum= 0;
	csum += command+size;
	int i;
	for(i=0; i<strlen(message); i++)
	{
		*(write_buffer+i+3)= *(message+i);    /* 4°-32° Byte Payload */
		csum += *(message+i);                 /* Checksum processing (no carry)  */
	}
	#ifdef DEBUG
		printf("%x\n",csum);
	#endif
	if(csum>>8) csum++;                     /* Checksum carry added if present */
	#ifdef DEBUG
		printf("  Calcolo checksum (carry): %hhx\n",(char)csum);
	#endif
	 /* truncate 1 byte, Little endian system (intel processor) */
	*(write_buffer+1)= (char)csum;          /* Checksum stored in the packet   */


	/*-----------------------------Packet sending-------------------------------------------------------*/
	int bytes_written= 0;  	/* Value for storing the number of bytes written to the port            */
	bytes_written= write(fd,write_buffer, MAX_WRITE);   /*     use write() to send data to port          */
						           /* "fd"                   - file descriptor pointing to the opened serial port */
									     /*	"write_buffer"         - address of the buffer containing data	            */
									     /* "MAX_WRITE" - No of bytes to write                               */
	#ifdef DEBUG
		printf("\n  %s written to ttyACM0",write_buffer);
		printf("\n");

		int h;
		for(h=0;h<9;h++)
			printf("%x\n",write_buffer[h]);

		printf("\n  %d Bytes written to ttyACM0", bytes_written);
	#endif
	printf("\n +----------------------------------+\n\n");


	/*-----------------------------Receiving response-------------------------------------------------------*/
	/* // NOT WORKING WITHOUT OPENING CUTECOM FIRST
	printf("  Receiving data... \n");

	char read_buffer[MAX_READ];   // Buffer to store the data received
	int  bytes_read = 0;    // Number of bytes read by the read() system call
	int j = 0;

	bytes_read = read(fd,&read_buffer,MAX_READ); // Read the data

	printf("\n\n  Bytes Rxed: %d", bytes_read); // Print the number of bytes read
	printf("\n\n  ");

	for(j=0;j<bytes_read;j++)	 // printing only the received characters
		printf("%c",read_buffer[j]);
	printf("\n +----------------------------------+\n\n\n");
	*/
	close(fd);/* Close the Serial port */

}
