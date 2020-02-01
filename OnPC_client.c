/*====================================================================================================*/
        /* Serial Port Programming in C (Serial Port Write)                                           */
	      /* Non Cannonical mode                                                                        */
/*----------------------------------------------------------------------------------------------------*/
        /* Program writes a character to the serial port at 19200 bps 8N1 format                      */
	      /* Baudrate - 19200                                                                           */
	      /* Stop bits - 1                                                                              */
	      /* No Parity                                                                                  */
/*----------------------------------------------------------------------------------------------------*/
  /* termios structure -  /usr/include/asm-generic/termbits.h    */
	/* use "man termios" to get more info about  termios structure */
	/*-------------------------------------------------------------*/

#include <stdio.h>
#include <string.h> //Necessary?
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */

#define MAX_BUF 256

void main(void){

	int fd;            /*File Descriptor*/

	printf("\n +----------------------------------+");
	printf("\n |        Serial Port Write         |");
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


	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
                                  /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode            */

	SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
    printf("\n  ERROR ! in Setting attributes");
	else
    printf("\n  BaudRate = 19200 \n  StopBits = 1 \n  Parity   = none\n");

/*------------------------------- Write data to serial port -----------------------------*/

//    char write_buffer[MAX_BUF];
//    scanf("%s",write_buffer);

  char write_buffer[] = "6codice";	/* Buffer containing characters to write into port	     */
	int  bytes_written  = 0;  	/* Value for storing the number of bytes written to the port */

	bytes_written = write(fd,write_buffer, sizeof(write_buffer));  //   /* use write() to send data to port     */
						           /* "fd"                   - file descriptor pointing to the opened serial port */
									     /*	"write_buffer"         - address of the buffer containing data	            */
									     /* "sizeof(write_buffer)" - No of bytes to write                               */
	printf("\n  %s written to ttyACM0",write_buffer);
	printf("\n  %d Bytes written to ttyACM0", bytes_written);
	printf("\n +----------------------------------+\n\n");

	close(fd);/* Close the Serial port */

}
