#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	       */
#include <errno.h>   /* ERROR Number Definitions           */

#define MAX_WRITE 32
#define MAX_READ 2
#define MAX_PAYLOAD 29
#define INPUT_FAIL 0x20
#define TO_INT_MASK 0x30
#define MAX_DEVICES 8
#define NO_DEV 0x4E
#define TEMP_SENSOR 0x53
#define LED 0x4C

#define CMD_START 0x01
#define CMD_ASSIGN 0x02
#define CMD_READINGS 0x03
#define CMD_LED_SETUP 0x04
#define CMD_TEMP_SETUP 0x05

#define CMD_ASSIGN_SIZE 0x02
#define CMD_LED_SETUP_SIZE 0x02

int SetSerialSettings(int fd, int speed, int parity)
{
	struct termios SerialPortSettings;	/* Create the structure                          */

	if(tcgetattr(fd, &SerialPortSettings) != 0)	/* Get the current attributes of the Serial port */
	{
		printf("\n ERROR getting attributes");
    return -1;
	}

	cfsetispeed(&SerialPortSettings,speed); /* Set Read  Speed as 19200                       */
	cfsetospeed(&SerialPortSettings,speed); /* Set Write Speed as 19200                       */
	cfmakeraw(&SerialPortSettings);

	SerialPortSettings.c_cflag &= ~(PARENB | PARODD);   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag |= parity;
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	   /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
	{
		printf("\n ERROR setting attributes!");
    return -1;
	}
	else
	{
		#ifdef DEBUG
			printf("\n  BaudRate = 19200 \n  StopBits = 1 \n  Parity   = none\n\n");
		#endif
		return 0;
	}
}

/* Packet assembling and sending */
int PckSend(int fd,unsigned char command, unsigned char size, unsigned char* message)
{
	unsigned char write_buffer[MAX_WRITE]= {0};

	unsigned char sizeCopy= size;   /* for printing purposes */
	#ifdef DEBUG
		printf("\n  Command(debug): %x\n",command);
		printf("  size: %x\n",size);
	#endif

	/*-----sizeControl(a size-field-only checksum) loaded in the last 3 bits of size (bit stealing)-------*/
	char sizeControl= 0;
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
	size <<= 3;                  /* size field only needs 5 bits (29 is max value) */
	size |= sizeControl;


	/* Checksum processing */
	unsigned short csum= 0;
	csum += command+size;
	int i;
	for(i=0; i<sizeCopy; i++)
	{
		*(write_buffer+i+3)= *(message+i);    /* 4°-32° Byte Payload writing */
		csum += *(message+i);                 /* Checksum processing (no carry)  */
	}
	#ifdef DEBUG
		printf("  Calcolo checksum (no carry): %x\n",csum);
	#endif
	if(csum>>8) csum++;                     /* Checksum carry added if present */
	#ifdef DEBUG
		printf("  Calcolo checksum (carry): %hhx\n",(char)csum);
	#endif

	/*-------------------------Final packet assembling------------------------------------------------ */
	*(write_buffer+0)= command;             /* 1° Byte Command  */
	/* truncate 1 byte, Little endian system (intel processor) */
	*(write_buffer+1)= (char)csum;          /* 2° Byte Checksum */
	*(write_buffer+2)= size;                /* 3° Byte Size     */

	/*-----------------------------Packet sending-------------------------------------------------------*/
	int bytes_written= 0;  	/* Value for storing the number of bytes written to the port */
	bytes_written= write(fd,write_buffer, MAX_WRITE);
						           /* "fd"                   - file descriptor pointing to the opened serial port */
									     /*	"write_buffer"         - address of the buffer containing data	            */
									     /* "MAX_WRITE" - No of bytes to write                                          */
	#ifdef DEBUG
		printf("\n  Bytes: [ ");
		int h;
		for(h=0;h<bytes_written;h++)
			printf("%c ",write_buffer[h]);

		printf("] written to ttyACM0\n");
		printf("\n  Bytes(hex): [ ");
		for(h=0;h<bytes_written;h++)
			printf("%x ",write_buffer[h]);

		printf("] written to ttyACM0\n");
		printf("\n  %d Bytes written to ttyACM0\n\n", bytes_written);
	#endif

	if(bytes_written <= 0) return -1;
	else return 0;
}

void printMenu(char *controller)
{
	printf("\n +----------------------------------------------------------------------------------------------+\n\n");
	printf("    Connected to: %s\n\n",controller);
	printf("    You can perform the following actions(select one):\n\n");

	printf("    1- Query available ports\n");
	printf("    2- Assign a new device(temp_sensor or led) to an available port\n");
	printf("    3- Setup an installed led\n");
	printf("    4- Setup an installed temp sensor\n");
	printf("    5- Get readings from installed temp sensors\n");

	printf("    9- Restart the configuration process(all data will be lost)\n");
	printf("    0- Exit the application\n\n");
}

void filterInput(char* input, int filter)
{
	fgets(input,filter+1,stdin);
	if(!strchr(input, '\n'))         // if string entered is longer than 1 char, input doesn't contain '\n'
		while(fgetc(stdin)!='\n')      // discard until newline, empty stdin buffer
	  	*input = INPUT_FAIL;         // inside the while a string with 2+ characters was passed
}

void queryChannels(unsigned char* ports, char* devices)
{
	printf("\n\n    Port-Device assignments(S= sensor, L=led, N= not assigned):\n\n");
	printf("    [");
	int i;
	for(i=0; i<MAX_DEVICES; i++)
			printf(" Port[%d]-Dev[%c]",i,*(devices+i));
	printf(" ]\n\n");
	printf("    Ports not represented are not assigned.\n");
}

int assignToPort(int fd, unsigned char* ports, char* devices)
{
	printf("    To which port are you connecting the device(0-7)?\n");
	printf("    0= pin2, 1= pin3, 2= pin5,  3= pin6\n");
	printf("    4= pin7, 5= pin8, 6= pin11, 7= pin12\n\n");
	char port[1];
	do
	{
		printf("    Port chosen: ");
		filterInput(port,sizeof(port));
	}while(((*port)-TO_INT_MASK >7) || ((*port)-TO_INT_MASK <0));
	printf("\n");
	/* if port chosen is already in use wait for confirmation new device was connected */
	if ((1<<((*port)-TO_INT_MASK)) & *ports) {
		printf("    The port chosen is already occupied. Be sure the new device is connected before going on\n");
		printf("    When you are ready press y|Y.\n\n");
		char ack[1];
		do
		{
			printf("    Ready? ");
			filterInput(ack,sizeof(ack));
		}while(*ack != 0x59 && *ack != 0x79);  /* *ack must be y or Y */
		printf("\n");
	}

	printf("    Which kind of device was connected?\n\n");
	printf("    1- Temperature sensor");
	printf("    2- LED\n\n");
	char dev[1];
	do
	{
		printf("    Device chosen: ");
		filterInput(dev,sizeof(dev));
	}while(((*dev)-TO_INT_MASK >2) || (*dev)-TO_INT_MASK <1);
	printf("\n");

	unsigned char command= CMD_ASSIGN;
	unsigned char size= CMD_ASSIGN_SIZE;
	unsigned char payload[CMD_ASSIGN_SIZE]= {0};
	if((*dev)-TO_INT_MASK == 1)
	{
		/* send sensor packet */
		*payload= TEMP_SENSOR;
		*(payload+1)= *port;
		if(PckSend(fd,command,size,payload) != 0)
		{
			printf("\n ERROR sending assignToPort() packet");
			return -1;
		}
	}
	else
	{
		/* send LED packet */
		*payload= LED;
		*(payload+1)= *port;
		if(PckSend(fd,command,size,payload) != 0)
		{
			printf("\n ERROR sending assignToPort() packet");
			return -1;
		}
	}
	#ifdef DEBUG
		printf(" Ports before new assign(debug): %x\n",*ports); //bug assegnare a porta 1 cancella il dev in porta 0
		printf(" Devices before(debug): [");
		int i;
		for(i=0; i<MAX_DEVICES; i++) printf(" %c",*(devices+i));
		printf(" ]\n");
	#endif
	*ports= (*ports | (1<<((*port)-TO_INT_MASK)));
	(*dev)--;
	*(devices + ((*port)-TO_INT_MASK))= ((*dev)-TO_INT_MASK) ? LED : TEMP_SENSOR;
	#ifdef DEBUG
		printf(" Ports after new assign(debug): %x\n",*ports);
		printf(" Devices after(debug): [");
		int j;
		for(j=0; j<MAX_DEVICES; j++) printf(" %c",*(devices+j));
		printf(" ]\n");
	#endif
	return 0;
}

int setLED(int fd, unsigned char* ports, char* devices)
{
	printf("    Which LED do you want to setup? (select a port: 0-7)\n");
	printf("    0= pin2, 1= pin3, 2= pin5,  3= pin6\n");
	printf("    4= pin7, 5= pin8, 6= pin11, 7= pin12\n\n");
	char port[1];
	do
	{
		printf("    Port chosen: ");
		filterInput(port,sizeof(port));
	}while(((*port)-TO_INT_MASK >7) || ((*port)-TO_INT_MASK <0));
	printf("\n");

	/* Check if a device, which is a LED, has been assigned to the port, else an error is reported */
	if( (((*ports) & (1<<((*port)-TO_INT_MASK))) == 0) || (*(devices + ((*port)-TO_INT_MASK))!= LED))
	{
		printf("\n No device or no LED is assigned to the port selected. ");
		printf("Assign a LED to a port before attempting again\n\n");
		return -1;
	}


	printf("    Choose a value to setup LED brightness level(0-255), using 3 digits(eg: 1=001):\n\n");
	char brightness[3]={0};
	do
	{
		printf("    Brightness level selected: ");
		filterInput(brightness,sizeof(brightness));
	}while(((*brightness) -TO_INT_MASK   >2) || ((*brightness)    -TO_INT_MASK   <0)
   || ((*(brightness+1))-TO_INT_MASK   >9) || ((*(brightness+1))-TO_INT_MASK   <0)
	 || ((*(brightness+2))-TO_INT_MASK   >9) || ((*(brightness+2))-TO_INT_MASK   <0)
	 || (((*brightness)-TO_INT_MASK ==2) && ((*(brightness+1))-TO_INT_MASK >5))
	 || (((*brightness)-TO_INT_MASK ==2) && ((*(brightness+1))-TO_INT_MASK ==5) && ((*(brightness+2))-TO_INT_MASK >5)));
	printf("\n");

	unsigned char command= CMD_LED_SETUP;
	unsigned char size= CMD_LED_SETUP_SIZE;
	unsigned char payload[CMD_LED_SETUP_SIZE]= {0};
	*payload= *port;
	*(payload+1)= ((*(brightness))  -TO_INT_MASK)*100
						   +((*(brightness+1))-TO_INT_MASK)*10
	           	 +((*(brightness+2))-TO_INT_MASK)*1;

	#ifdef DEBUG
		printf(" Brightness level(debug): %d\n\n",(int)(*(payload+1)));
	#endif
	if(PckSend(fd,command,size,payload) != 0)
	{
		printf("\n ERROR sending setLED() packet");
		return -1;
	}

	return 0;
}

int start(int fd, char* controller)
{
	printf("    Welcome to the configuration process, you need to choose a name for the controller first.\n");
	printf("    Name length must be up to 29 characters and cannot be void.\n\n");

	do
	{
		printf("    Controller Name: ");
		fgets(controller,MAX_PAYLOAD+1,stdin);
		#ifdef DEBUG
			printf("    Controller(debug): %s\n",controller);
			printf("    Size fgets(debug): %d\n\n",(int)strlen(controller));
		#endif
	}while(strlen(controller)<=1);

	unsigned char command= CMD_START;
	unsigned char size= strlen(controller)-1;
	unsigned char *payload= (unsigned char*)controller;

	if(PckSend(fd,command,size,payload) != 0)
	{
		printf("\n ERROR sending start() packet");
		return -1;
	}
	return 0;
}

void save(char* controller) //da implementare
{

}

/* Perform selected action */
int selectAction(int fd, char* controller, unsigned char* ports, char* devices, char action)
{
	switch(action)
	{
		case 1:
			queryChannels(ports,devices);
			return 0;
		case 2:
			if(assignToPort(fd,ports,devices) != 0)
			{
				printf("\n ERROR encountered calling assignToPort()");
			}
			return 0;
		case 3:
			if(setLED(fd,ports,devices) != 0)
			{
				printf("\n ERROR encountered calling setLED()");
			}
			return 0;
		case 4:
			return 0;
		case 5:
			return 0;
		case 9:
			if(start(fd,controller) != 0)
			{
				printf("\n ERROR encountered calling start()");
			}
			return 0;
		case 0:
			save(controller);
			exit(0);

		default:
			printf(" ERROR invalid input, select one of the available actions (1-5)\n\n");
			return -1;
	}
}

void load(char* controller) //da implementare controller lo uso per caricarci il nome salvato sul file
{

}

char controller[MAX_PAYLOAD];   /* store controller name */
unsigned char ports= 0;         /* keeps track of ports, 1bit per port  */
char devices[MAX_DEVICES]= {NO_DEV,NO_DEV,NO_DEV,NO_DEV,
									          NO_DEV,NO_DEV,NO_DEV,NO_DEV}; /* keeps track of devices assigned to ports */

int main(void)
{
	printf("\n +----------------------------------------------------------------------------------------------+");
	printf("\n |                                 SmartHouse: Configuration                                    |");
	printf("\n +----------------------------------------------------------------------------------------------+\n\n");

	/*---------------------- Opening the Serial Port ---------------------------*/
	int fd;  /*File Descriptor*/
  fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY | O_SYNC );
      /* /dev/ttyACM0 is the virtual file for the Arduino Board   */
			/* O_RDWR Read/Write access to serial port           */
			/* O_NOCTTY - No terminal will control the process   */
	if(fd == -1)						/* Error Checking */
	{
		printf("\n ERROR opening ttyACM0  ");
	}
  else
	{
		#ifdef DEBUG
			printf("\n  ttyACM0 Opened Successfully ");
		#endif
	}
	if(SetSerialSettings(fd,B19200,0) != 0)
	{
		printf("\n ERROR during serial setup\n");
		exit(1);
	}
	/*--------------------------------------------------------------------------*/

	if(start(fd,controller) != 0)
	{
		printf("\n ERROR encountered calling start()");
	}
	while(1)
	{
		printMenu(controller);
		char sel[1];
		do
		{
			printf("    Entered: ");
			filterInput(sel,sizeof(sel));
		}while(selectAction(fd,controller,&ports,devices,(*sel)-TO_INT_MASK) != 0);
		printf("\n");
	}

	/*-----------------------------Receiving response-------------------------------------------------------*/

	/*
	printf("  Receiving data... \n");

	char read_buffer[MAX_READ];   // Buffer to store the data received
	int  bytes_read = 0;    // Number of bytes read by the read() system call
	int j = 0;

	bytes_read = read(fd,&read_buffer,MAX_READ); // Read the data

	printf("\n\n  Bytes read: %d from ttyACM0", bytes_read); // Print the number of bytes read
	printf("\n\n  ");

	for(j=0;j<bytes_read;j++)	 // printing only the received characters
		printf("%c",read_buffer[j]);
	*/

	printf("\n +----------------------------------------------------------------------------------------------+\n\n");
	close(fd);   /* Close the Serial port */

}
