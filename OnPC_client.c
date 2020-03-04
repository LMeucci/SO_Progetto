#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	       */
#include <errno.h>   /* ERROR Number Definitions           */

#define MAX_SERIAL 8
#define MAX_CONTROLLER_NAME 20
#define MAX_DEV_TYPES 2
#define MAX_DEVICES 8
#define LOAD_SAVE_SIZE 36

#define INPUT_FAIL 0x10
#define YES 0x79
#define NO 0x6E
#define CHAR_TO_HEX 0x30
#define SIZECTL_MASK 0x07

#define NO_DEV 0x4E
#define PHOTORESISTOR 0x50
#define LED 0x4C

#define CMD_START 0x01
#define CMD_LED_SETUP 0x02
#define CMD_PR_SETUP 0x03
#define CMD_PR_READING 0x04
#define CMD_REMOVE_LED 0x05
#define CMD_REMOVE_PR 0x06

#define CMD_START_SIZE 0x05
#define CMD_LED_SETUP_SIZE 0x02
#define CMD_PR_SETUP_SIZE 0x02
#define CMD_PR_READING_SIZE 0x01
#define CMD_REMOVE_LED_SIZE 0x01
#define CMD_REMOVE_PR_SIZE 0x01

#define RSP_ACK 0x41
#define RSP_NACK 0x4E


typedef struct SmartHouse
{
	int fd;
	char controller[MAX_CONTROLLER_NAME];
	char devLED[MAX_DEVICES];
	char devPR[MAX_DEVICES];
	unsigned char lastPckSent[MAX_SERIAL];
	unsigned char lastPckReceived[MAX_SERIAL];
}SmartHouse;

/* SmartHouse declaration */
SmartHouse sh;

const char pins[MAX_DEV_TYPES][MAX_DEVICES]= {{0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7},
                                              {0x02,0x03,0x05,0x06,0x07,0x08,0x0b,0x0c}};

void smartHouseInit(SmartHouse *sh, int descriptor)
{
	sh->fd= descriptor;
	*sh->controller= '\0';    //empty string when attempting to print it
	int i;
	for(i=0; i<MAX_DEVICES; i++)
	{
		sh->devLED[i]= NO_DEV;
		sh->devPR[i]= NO_DEV;
		sh->lastPckSent[i]= 0;
		sh->lastPckReceived[i]= 0;
	}
}

int setSerialSettings(int fd, int speed, int parity)
{
	struct termios SerialPortSettings;	/* Create the structure                          */

	if(tcgetattr(fd, &SerialPortSettings) != 0)	/* Get the current attributes of the Serial pin */
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

/*-----------------------Packets handling functions---------------------------------------------*/
int pckSend(SmartHouse *sh, unsigned char command, unsigned char size, unsigned char* message)
{
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
	size <<= 3;                  /* space for sizeControl bits */
	size |= sizeControl;


	/* Checksum processing */
	unsigned short csum= 0;
	csum += command+size;
	int i;
	for(i=0; i<sizeCopy; i++)
	{
		*((sh->lastPckSent)+i+3)= *(message+i);    /* 4°-32° Byte Payload writing     */
		csum += *(message+i);                      /* Checksum processing (no carry)  */
	}
	#ifdef DEBUG
		printf("  Calcolo checksum (no carry): %x\n",csum);
	#endif
	if(csum>>8) csum++;                     /* Checksum carry added if present */
	#ifdef DEBUG
		printf("  Calcolo checksum (carry): %hhx\n",(char)csum);
	#endif

	/*-------------------------Final packet assembling------------------------------------------------ */
	*((sh->lastPckSent)+0)= command;             /* 1° Byte Command  */
	/* truncate 1 byte, Little endian system (intel processor) */
	*((sh->lastPckSent)+1)= (char)csum;          /* 2° Byte Checksum */
	*((sh->lastPckSent)+2)= size;                /* 3° Byte Size     */

	/*-----------------------------Packet sending-------------------------------------------------------*/
	int bytesWritten= 0;  	/* Value for storing the number of bytes written to the pin */
	bytesWritten= write(sh->fd,(sh->lastPckSent), MAX_SERIAL);
						           /* "fd"                   - file descriptor pointing to the opened serial pin */
									     /*	"(sh->lastPckSent)"         - address of the buffer containing data	        */
									     /* "MAX_SERIAL" - No of bytes to write                                         */
	#ifdef DEBUG
		printf("\n  Bytes: [ ");
		int h;
		for(h=0;h<bytesWritten;h++)
			printf("%c ",(sh->lastPckSent)[h]);

		printf("] written to ttyACM0\n");
		printf("\n  Bytes(hex): [ ");
		for(h=0;h<bytesWritten;h++)
			printf("%x ",(sh->lastPckSent)[h]);

		printf("] written to ttyACM0\n");
		printf("\n  %d Bytes written to ttyACM0\n\n", bytesWritten);
	#endif

	if(bytesWritten <= 0) return -1;
	else return 0;
}

int pckReceive(SmartHouse *sh)
{
	#ifdef DEBUG
		printf("\n  Receiving data... ");
	#endif

	char bufferRead[MAX_SERIAL]= {0};
	int toRead= MAX_SERIAL;
	int prevReading= 0;
	do
	{
		int  bytesRead = 0;    // Number of bytes read by the read() system call
		bytesRead = read(sh->fd,bufferRead,toRead); // Read the data

		#ifdef DEBUG
			printf("\n\n  Bytes read: %d from ttyACM0", bytesRead); // Print the number of bytes read
			printf("\n\n  ");

			int j;
			for(j=0;j<bytesRead;j++)	 // printing only the received characters
				printf(" %x",(unsigned char)bufferRead[j]);
		#endif

		int i;
		for(i=0; i<bytesRead+prevReading; i++)
			sh->lastPckReceived[i+prevReading]= bufferRead[i];

		if(bytesRead <= 0) return -1;
		toRead-= bytesRead;
		prevReading+= bytesRead;
	}while(toRead>0);

	#ifdef DEBUG
		printf("\n\n  Last packet read(debug,hex): ");
		int k;
		for(k=0;k<MAX_SERIAL;k++)
			printf(" %x",(unsigned char)sh->lastPckReceived[k]);
		printf("\n  Last packet read(debug,char): ");
		for(k=0;k<MAX_SERIAL;k++)
			printf(" %c",sh->lastPckReceived[k]);
		printf("\n\n");
	#endif

	return 0;
}

int pckCheck(SmartHouse *sh)
{
	/* Check if ack has been received */
	if(sh->lastPckReceived[0] != RSP_ACK)
	{
		printf("\n ERROR response packet was not an ack: ");
		if(sh->lastPckReceived[0] == RSP_NACK) printf("NACK received\n");
		else printf("Unknown response received\n");
		return -1;
	}
	/* Check if ack belongs to the last command sent */
	if(sh->lastPckReceived[2] != sh->lastPckSent[0])
	{
		printf("\n ERROR response packet doesn't belong to the last command issued, command[%x]\n",sh->lastPckReceived[2]);
		return -1;
	}

	/* Check control bits of size field */
  unsigned char sizeCtl= (sh->lastPckReceived[3]) & SIZECTL_MASK;
  unsigned char sizeControl= 0;
  unsigned char temp= sh->lastPckReceived[3] >>3;  /* discard 3 LSB to recover size */
	int k;
	for(k=0; k<5; k++){
		sizeControl += temp&1;      /* counting 1 bits among the first 5 bits of size */
		temp>>=1;
	}
  if(sizeCtl != sizeControl) return -1;

  /* Checksum processing */
  unsigned char rsp= sh->lastPckReceived[0];
  unsigned char cksum= sh->lastPckReceived[1];
	unsigned char cmd= sh->lastPckReceived[2];
  unsigned char sz= sh->lastPckReceived[3];

 	unsigned short checksum= 0;
	checksum += rsp+cmd+sz;
	int i;
	for(i=4; i<MAX_SERIAL; i++){
		checksum += sh->lastPckReceived[i];                /* Checksum processing (no carry)  */
	}
	if(checksum>>8) checksum++;                  /* Checksum carry added if present */
  if(cksum != (unsigned char)checksum)
	{
		printf("\n ERROR processing checksum, command[%x]\n",sh->lastPckReceived[2]);
		return -1;     /* truncate 1 byte, Little endian system (intel)*/
	}
  /* All checks passed */
	return 0;
}

void pckInit(SmartHouse *sh)
{
	int i;
	for(i=0;i<MAX_SERIAL;i++)
	{
		*((sh->lastPckSent)+i)= 0x00;
		*((sh->lastPckReceived)+i)= 0x00;
	}
}

int dataExchange(SmartHouse *sh, unsigned char command, unsigned char size, unsigned char* payload)
{
	int try=0;
	retry:
		if(pckSend(sh,command,size,payload) != 0)
		{
			printf("\n ERROR sending a packet, command[%x]\n",command);
			return -1;
		}
		if(pckReceive(sh) != 0)
		{
			printf("\n ERROR receiving a packet, command[%x]\n",command);
			return -1;
		}
		if(pckCheck(sh) != 0)
		{
			if(try++ <10)
			{
				pckInit(sh);
				goto retry;
			}
			printf("\n ERROR checking a packet, command[%x]\n",command);
			return -1;
		}

	return 0;
}
/*-----------------------Client functions------------------------------------------------------*/

void printMenu(SmartHouse *sh)
{
	printf("\n +----------------------------------------------------------------------------------------------+\n\n");
	printf("    Connected to: %s\n\n",sh->controller);
	printf("    You can perform the following actions(select one):\n\n");

	printf("    1- Query available ports\n");
	printf("    2- Install a LED\n");
	printf("    3- Install a photoresistor\n");
	printf("    4- Get a reading from an installed photoresistor\n");
	printf("    5- Uninstall a LED\n");
	printf("    6- Uninstall a photoresistor\n\n");

	printf("    9- Restart the configuration process\n");
	printf("    0- Exit the application\n\n");
}

void filterInput(char* input, int filter)
{
	fgets(input,filter+1,stdin);
	if(!strchr(input, '\n'))         // if string entered is longer than "filter" char, input doesn't contain '\n'
		while(fgetc(stdin)!='\n')      // discard until newline, empty stdin buffer
	  	*input = INPUT_FAIL;         // inside the while a string with "filter" or more characters was passed
}

void queryChannels(SmartHouse *sh)
{
	printf("\n\n    Port-Device assignments (P= photoresistor, L=led, N= not assigned):\n\n");

	printf("    LED pins\n");
	int i;
	for(i=0; i<MAX_DEVICES/2; i++)
			printf("	Pin[%d]-Dev[%c]",pins[1][i],*((sh->devLED)+i));
	printf("\n");
	for(; i<MAX_DEVICES; i++)
		printf("	Pin[%d]-Dev[%c]",pins[1][i],*((sh->devLED)+i));
	printf("\n\n");

	printf("    Photoresistor pins\n");
	for(i=0; i<MAX_DEVICES/2; i++)
			printf("	Pin[%x]-Dev[%c]",(unsigned char)pins[0][i],*((sh->devPR)+i));
	printf("\n");
	for(;i<MAX_DEVICES; i++)
			printf("	Pin[%x]-Dev[%c]",(unsigned char)pins[0][i],*((sh->devPR)+i));
	printf("\n\n");
}

int setLED(SmartHouse *sh)
{
	printf("    Which LED do you want to setup? (select a pin: 0-7)\n");
	printf("    0= pin2, 1= pin3, 2= pin5,  3= pin6\n");
	printf("    4= pin7, 5= pin8, 6= pin11, 7= pin12\n");
	printf("    Pins 4-7 have advanced LED features\n");
	char pin[1];
	do
	{
		printf("    Pin chosen: ");
		filterInput(pin,sizeof(pin));
		pin[0]-= CHAR_TO_HEX;
	}while(((*pin) >7) || ((*pin) <0));
	printf("\n");

	printf("    Choose a value to setup LED brightness level(0-255), using 3 digits(eg: 1=001):\n\n");
	char brightness[3]={0};
	do
	{
		printf("    Brightness level selected: ");
		filterInput(brightness,sizeof(brightness));
		brightness[0]-= CHAR_TO_HEX;
		brightness[1]-= CHAR_TO_HEX;
		brightness[2]-= CHAR_TO_HEX;
	}while(((*brightness)  >2) || ((*brightness)    <0)
   || ((*(brightness+1)) >9) || ((*(brightness+1)) <0)
	 || ((*(brightness+2)) >9) || ((*(brightness+2)) <0)
	 || (((*brightness) ==2) && ((*(brightness+1)) >5))
	 || (((*brightness) ==2) && ((*(brightness+1)) ==5) && ((*(brightness+2)) >5)));
	printf("\n");

	unsigned char payload[CMD_LED_SETUP_SIZE]= {0};
	*payload= *pin;
	*(payload+1)= (*(brightness)  )*100
						   +(*(brightness+1))*10
	           	 +(*(brightness+2))*1;
	#ifdef DEBUG
		printf(" Brightness level(debug): %d\n\n",(int)(*(payload+1)));
	#endif

	/* send set_led packet */
	if(dataExchange(sh,CMD_LED_SETUP,CMD_LED_SETUP_SIZE,payload) != 0)
	{
		printf("\n ERROR during data exchange, command[%x]\n",CMD_LED_SETUP);
		return -1;
	}

	/* erase last packet sent & last packet received, current task is complete */
	pckInit(sh);
	/* register new LED */
	sh->devLED[(int)*pin]= LED;
	return 0;
}

int setPR(SmartHouse *sh)
{
	printf("    Which photoresistor do you want to setup? (select a pin: 0-7)\n");
	printf("    0= pinA0, 1= pinA1, 2= pinA2,  3= pinA3\n");
	printf("    4= pinA4, 5= pinA5, 6= pinA6, 7= pinA7\n");
	char pin[1];
	do
	{
		printf("    Pin chosen: ");
		filterInput(pin,sizeof(pin));
		pin[0]-= CHAR_TO_HEX;
	}while(((*pin) >7) || ((*pin) <0));
	printf("\n");

	printf("    Select if you want to assign this photoresistor to a pwm LED or the default LED.\n");
	printf("    0= pin2, 1= pin3, 2= pin5,  3= pin6\n");
	printf("    4= pin7, 5= pin8, 6= pin11, 7= pin12\n\n");

	printf("    0-7) for a pwm LED pin\n");
	printf("    8) default LED\n\n");
	char ledPin[1];
	do
	{
		printf("    Pin chosen: ");
		filterInput(ledPin,sizeof(ledPin));
		ledPin[0]-= CHAR_TO_HEX;
	}while(((*ledPin) >8) || ((*ledPin) <0));
	printf("\n");

	if( ((*ledPin) <8) && (sh->devLED[(int)(*ledPin)] != LED) )
	{
		printf("\n ERROR pwm LED selected not set up yet");
		return -1;
	}

	unsigned char payload[CMD_PR_SETUP_SIZE]= {0};
	*payload= *pin;
	*(payload+1)= *ledPin;

	/* send set_photoresistor packet */
	if(dataExchange(sh,CMD_PR_SETUP,CMD_PR_SETUP_SIZE,payload) != 0)
	{
		printf("\n ERROR during data exchange, command[%x]\n",CMD_PR_SETUP);
		return -1;
	}

	/* erase last packet sent & last packet received, current task is complete */
	pckInit(sh);
	/* register new photoresistor */
	sh->devPR[(int)(*pin)]= PHOTORESISTOR;
	return 0;
}

int getPRreading(SmartHouse *sh)
{
	printf("    Which photoresistor do you want to read? (select a pin: 0-7)\n");
	printf("    0= pinA0, 1= pinA1, 2= pinA2,  3= pinA3\n");
	printf("    4= pinA4, 5= pinA5, 6= pinA6, 7= pinA7\n");
	char pin[1];
	do
	{
		printf("    Pin chosen: ");
		filterInput(pin,sizeof(pin));
		pin[0]-= CHAR_TO_HEX;
	}while(((*pin) >7) || ((*pin) <0));
	printf("\n");

	if( sh->devPR[(int)(*pin)] != PHOTORESISTOR )
	{
		printf("\n ERROR photoresistor selected not set up yet");
		return -1;
	}

	/* send get_photoresistor_reading packet */
	if(dataExchange(sh,CMD_PR_READING,CMD_PR_READING_SIZE,(unsigned char*)pin) != 0)
	{
		printf("\n ERROR during data exchange, command[%x]\n",CMD_PR_READING);
		return -1;
	}
	/* preparing result to print it */
	unsigned short valueRead= 0;
	valueRead |= sh->lastPckReceived[5];
	valueRead <<= 8;
	valueRead |= sh->lastPckReceived[4];
	printf("\n    Current photoresistor value on the adc pin[%x]: %hu\n\n",(unsigned char)pins[0][(int)(*pin)],valueRead);
	/* erase last packet sent & last packet received, current task is complete */
	pckInit(sh);
	return 0;
}

int removeLED(SmartHouse *sh)
{
	printf("    To which pin belongs the device(0-7)?\n");
	printf("    0= pin2, 1= pin3, 2= pin5,  3= pin6\n");
	printf("    4= pin7, 5= pin8, 6= pin11, 7= pin12\n");
	char pin[1];
	do
	{
		printf("    Pin chosen: ");
		filterInput(pin,sizeof(pin));
		pin[0]-= CHAR_TO_HEX;
	}while(((*pin) >7) || ((*pin) <0));
	printf("\n");

	/* send remove packet */
	if(dataExchange(sh,CMD_REMOVE_LED,CMD_REMOVE_LED_SIZE,(unsigned char*)pin) != 0)
	{
		printf("\n ERROR during data exchange, command[%x]\n",CMD_REMOVE_LED);
		return -1;
	}
	/* erase last packet sent & last packet received, current task is complete */
	pckInit(sh);
	/* unregister old LED */
	sh->devLED[(int)*pin]= NO_DEV;
	return 0;
}

int removePR(SmartHouse *sh)
{
	printf("    To which pin belongs the device(0-7)?\n");
	printf("    0= pinA0, 1= pinA1, 2= pinA2, 3= pinA3\n");
	printf("    4= pinA4, 5= pinA5, 6= pinA6, 7= pinA7\n");
	char pin[1];
	do
	{
		printf("    Pin chosen: ");
		filterInput(pin,sizeof(pin));
		pin[0]-= CHAR_TO_HEX;
	}while(((*pin) >7) || ((*pin) <0));
	printf("\n");

	/* send remove packet */
	if(dataExchange(sh,CMD_REMOVE_PR,CMD_REMOVE_PR_SIZE,(unsigned char*)pin) != 0)
	{
		printf("\n ERROR during data exchange, command[%x]\n",CMD_REMOVE_PR);
		return -1;
	}
	/* erase last packet sent & last packet received, current task is complete */
	pckInit(sh);
	/* unregister old photoresistor */
	sh->devPR[(int)(*pin)]= NO_DEV;
	return 0;
}

int start(SmartHouse *sh)
{
	printf("    Welcome to the configuration process, you need to choose a name for the controller first.\n");
	printf("    Name length must be up to 20 characters and cannot be void.\n\n");

	do
	{
		printf("    Controller Name: ");
		filterInput(sh->controller, MAX_CONTROLLER_NAME);
	}while(strlen(sh->controller)<=1 || (*(sh->controller))== INPUT_FAIL);
	printf("\n");
	#ifdef DEBUG
		printf("    Controller(debug): %s\n",sh->controller);
		printf("    Size fgets(debug): %d\n\n",(int)strlen(sh->controller));
	#endif

	/* send start packet */
	if(dataExchange(sh,CMD_START,CMD_START_SIZE,(unsigned char*)"HELLO") != 0)
	{
		printf("\n ERROR during data exchange, command[%x]\n",CMD_START);
	}
	/* erase last packet sent & last packet received, current task is complete */
	pckInit(sh);
	return 0;
}

int save(SmartHouse *sh)
{
	printf("    Do you want to save current configuration? (y/n)\n");
	char choice[1];
	do
	{
		printf("    Input: ");
		filterInput(choice,sizeof(choice));
	}while(((*choice) != YES) && ((*choice) != NO));
	printf("\n");

	if((*choice) == YES)
	{
		printf("    Saving current configuration...\n");
		FILE *fileToSave;
		fileToSave= fopen("controller.bin","w+b");
		if(fileToSave == NULL)
		{
			return -1;
		}
		/* saving configuration */
		char buffer[LOAD_SAVE_SIZE];
		int i;
		for(i=0; i<8; i++)
			buffer[i]= 	sh->devLED[i];
		for(i=0; i<8; i++)
			buffer[i+8]= sh->devPR[i];
		for(i=0; i<20; i++)
			buffer[i+16]= sh->controller[i];

		fwrite(buffer,1,sizeof(buffer),fileToSave);
		fclose(fileToSave);
		printf("    DONE.\n\n");
	}
	return 0;
}

/* Perform selected action */
int selectAction(SmartHouse *sh, char action)
{
	switch(action)
	{
		case 1:
			queryChannels(sh);
			return 0;
		case 2:
			if(setLED(sh) != 0)
			{
				printf("\n ERROR encountered during setLED()\n\n");
			}
			return 0;
		case 3:
			if(setPR(sh) != 0)
			{
				printf("\n ERROR encountered during setPR()\n\n");
			}
			return 0;
		case 4:
			if(getPRreading(sh) != 0)
			{
				printf("\n ERROR encountered during getPRreading()\n\n");
			}
			return 0;
		case 5:
			if(removeLED(sh) != 0)
			{
				printf("\n ERROR encountered during removeLED()\n\n");
			}
			return 0;
		case 6:
			if(removePR(sh) != 0)
			{
				printf("\n ERROR encountered during removePR()\n\n");
			}
			return 0;
		case 9:
			if(start(sh) != 0)
			{
				printf("\n ERROR encountered during start()\n\n");
			}
			return 0;
		case 0:
			if(save(sh) != 0)
			{
				printf("\n ERROR encountered during save()\n\n");
			}
			exit(0);

		default:
			printf(" ERROR invalid input, select one of the available actions (1-5)\n\n");
			return -1;
	}
}

int load(SmartHouse *sh)
{
	FILE *savedFile;
	savedFile= fopen("controller.bin", "rb");
	if(savedFile == NULL)
	{
		if(start(sh) != 0)
		{
			printf("\n ERROR during start()\n\n");
			return -1;
		}
	}
	else
	{
		printf("    A previous configuration was saved.");
		printf(" Do you want to load last configuration? (y/n)\n");
		char choice[1];
		do
		{
			printf("    Input: ");
			filterInput(choice,sizeof(choice));
		}while(((*choice) != YES) && ((*choice) != NO));
		printf("\n");

		if((*choice) == YES)
		{
			printf("    Loading previous configuration...\n");
			char buffer[LOAD_SAVE_SIZE+1]; // +1 to deal with fgets termination char
			fgets(buffer,LOAD_SAVE_SIZE+1,savedFile);
			int i;
			for(i=0; i<8; i++)
				sh->devLED[i]= buffer[i];
			for(i=0; i<8; i++)
				sh->devPR[i]= buffer[i+8];
			for(i=0; i<20; i++)
				sh->controller[i]= buffer[i+16];

			fclose(savedFile);
			printf("    DONE.\n\n");
		}
		else
		{
			if(start(sh) != 0)
			{
				printf("\n ERROR during start()\n\n");
				return -1;
			}
		}
	}

	return 0;
}


int main(void)
{
	printf("\n");
	printf("\n +----------------------------------------------------------------------------------------------+");
	printf("\n |                                 SmartHouse: Configuration                                    |");
	printf("\n +----------------------------------------------------------------------------------------------+\n\n");

	/*---------------------- Opening the Serial Port ---------------------------*/
	int fd;  /*File Descriptor*/
  fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY | O_SYNC );
      /* /dev/ttyACM0 is the virtual file for the Arduino Board   */
			/* O_RDWR Read/Write access to serial pin           */
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
	if(setSerialSettings(fd,B19200,0) != 0)
	{
		printf("\n ERROR during serial setup\n");
		exit(1);
	}
	/*--------------------------------------------------------------------------*/

	smartHouseInit(&sh,fd);
	if(load(&sh) != 0)
	{
		printf("\n ERROR during load()\n\n");
		exit(-1);
	}

	while(1)
	{
		printMenu(&sh);
		char sel[1];
		do
		{
			printf("    Entered: ");
			filterInput(sel,sizeof(sel));
		}while(selectAction(&sh,(*sel)-CHAR_TO_HEX) != 0);
		printf("\n");
	}

	printf("\n +----------------------------------------------------------------------------------------------+\n\n");
	close(fd);   /* Close the Serial pin */

}
