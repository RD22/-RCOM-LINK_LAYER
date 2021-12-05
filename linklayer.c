#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include "linklayer.h"

#define FALSE 0
#define TRUE 1

#define FLAG 0x7E
#define ESCAPE_FLAG 0x7D
#define ADD1 0x05
#define ADD2 0x02
#define CDISC 0x0A
#define CSET 0x03
#define CUA 0x07
#define BUF_SIZE 256
#define SEQ1 0x00
#define SEQ2 0x02
#define ACK1 0x01
#define ACK2 0x21
#define REJ1 0X05
#define REJ2 0x25

char prevAckN = 0x01; //RR Tracker
char prevSeqN = 0x02; //SEQ Number Tracker
int fd;
struct termios oldtio;
struct termios newtio;
int send = 1; //Binary resend flag
int alarmEnabled = FALSE;
int alarmCount = 0;
int input;
char currChar[MAX_PAYLOAD_SIZE];
char singleChar;
int TotalAlarms = 0;  //Number of Alarms/ Retransmissions by Transmitter
int TotalRejects = 0; //Number of Rejects by Receiver
struct linkLayer ll;

void alarmHandler(int sig)
{
	alarmEnabled = FALSE;
	alarmCount++;
	TotalAlarms++;
	send = 1;
	printf("Alarm #%d\n", alarmCount);
	if (alarmCount >= ll.numTries)
	{
		printf("Too many timeouts/retransmissions. Connection failed\n");
		exit(-1);
	}
}

int stuff_byte(char inp[], char stuff[], int packetSize)
{
	int i, j = 0;

	for (i = 0; i < packetSize; i++)
	{
		if (i == 0 || i == packetSize - 1)
		{ //If its the first or last byte, ignore, as they are the flag
			stuff[j++] = inp[i];
			continue;
		}
		if (inp[i] == ESCAPE_FLAG || inp[i] == FLAG)
		{ //If FLAG or ESCAPEFLAG, replace with escape flag followed by byte xor'ed with 0x20
			stuff[j++] = ESCAPE_FLAG;
			stuff[j++] = 0x20 ^ inp[i];
		}
		else
		{
			stuff[j++] = inp[i];
		}
	}

	return j;
}

int destuff_byte(char stuff[], char destuff[], int packetSize)
{
	int i, j = 0;

	for (i = 0; i < packetSize; i++)
	{
		if (i == 0 || i == packetSize - 1)
		{ //If its the first or last byte, ignore, as they are the flag
			destuff[j++] = stuff[i];
			continue;
		}

		if (stuff[i] == ESCAPE_FLAG)
		{ //If ESCAPEFLAG, destuff
			i++;
			destuff[j++] = (stuff[i] ^ 0x20);
		}
		else
		{
			destuff[j] = stuff[i];
			j++;
		}
	}
	return j;
}

int llopen(linkLayer parameters)
{

	fd = open(parameters.serialPort, O_RDWR | O_NOCTTY);
	if (fd < 0)
	{ //Error with Serial Port
		perror(parameters.serialPort);
		return -1;
	}

	(void)signal(SIGALRM, alarmHandler);

	// Save current port settings
	if (tcgetattr(fd, &oldtio) == -1)
	{ //Error getting attributes
		perror("tcgetattr");
		return -1;
	}

	// Clear struct for new port settings
	bzero(&newtio, sizeof(newtio));

	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;

	switch (parameters.baudRate)
	{ //BAUDRATE flag selection
	case 115200:
		newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
		break;
	case 57600:
		newtio.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
		break;
	case 38400:
		newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
		break;
	case 19200:
		newtio.c_cflag = B19200 | CS8 | CLOCAL | CREAD;
		break;
	case 9600:
		newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
		break;
	case 4800:
		newtio.c_cflag = B4800 | CS8 | CLOCAL | CREAD;
		break;
	case 2400:
		newtio.c_cflag = B2400 | CS8 | CLOCAL | CREAD;
		break;
	case 1800:
		newtio.c_cflag = B1800 | CS8 | CLOCAL | CREAD;
		break;
	case 1200:
		newtio.c_cflag = B1200 | CS8 | CLOCAL | CREAD;
		break;
	case 600:
		newtio.c_cflag = B600 | CS8 | CLOCAL | CREAD;
		break;
	case 300:
		newtio.c_cflag = B300 | CS8 | CLOCAL | CREAD;
		break;
	case 200:
		newtio.c_cflag = B200 | CS8 | CLOCAL | CREAD;
		break;
	case 150:
		newtio.c_cflag = B150 | CS8 | CLOCAL | CREAD;
		break;
	case 110:
		newtio.c_cflag = B110 | CS8 | CLOCAL | CREAD;
		break;
	case 75:
		newtio.c_cflag = B75 | CS8 | CLOCAL | CREAD;
		break;
	case 50:
		newtio.c_cflag = B50 | CS8 | CLOCAL | CREAD;
		break;
	default:
		newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
		break;
	}

	// Set input mode (non-canonical, no echo,...)
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 50; // Inter-character timer
	newtio.c_cc[VMIN] = 0;

	// Now clean the line and activate the settings for the port
	// tcflush() discards data written to the object referred  to
	// by  fd but not transmitted, or data received but not read,
	// depending on the value of queue_selector:
	//   TCIFLUSH - flushes data received but not read.
	tcflush(fd, TCIOFLUSH);

	// Set new port settings
	if (tcsetattr(fd, TCSANOW, &newtio) == -1)
	{
		perror("tcsetattr");
		exit(-1);
	}
	printf("New termios structure set\n");

	ll = parameters; //Save parameters as internal linklayer struct

	if (ll.role == 0)
	{ //If TRANSMITTER

		while (1)
		{ //Send SET

			char SET[] = {FLAG, ADD1, CSET, ADD1 ^ CSET, FLAG};
			if (send == 1)
			{
				write(fd, SET, 5);
				send = 0;
			}

			if (alarmEnabled == FALSE)
			{
				alarm(ll.timeOut); // Set alarm to be triggered in ll.timeOut seconds
				alarmEnabled = TRUE;
			}

			input = read(fd, &currChar, 5);
			alarm(0);

			if ((currChar[1] ^ currChar[2]) != (ADD1 ^ CUA))
				send = 1; //If there is an error in header, resend
			if (send == 0)
				break;
		}
		return 1;
	}

	else if (ll.role == 1)
	{ //If RECEIVER

		enum flag_state
		{
			START,
			FLAG_GOT,
			A_GOT,
			C_GOT,
			BCC_GOT,
			STOP
		};
		enum flag_state flag = START;

		while (flag != STOP)
		{ //Implementação da máquina de estados
			input = read(fd, &singleChar, 1);

			switch (singleChar)
			{
			case FLAG:
				if (flag == BCC_GOT)
					flag = STOP;
				else
					flag = FLAG_GOT;
				break;
			case ADD1:
				if (flag == FLAG_GOT)
					flag = A_GOT;
				else
					flag = START;
				break;
			case CSET:
				if (flag == A_GOT)
					flag = C_GOT;
				else
					flag = START;
				break;
			case ADD1 ^ CSET:
				if (flag == C_GOT)
					flag = BCC_GOT;
				else
					flag = START;
				break;
			default:
				flag = START;
				break;
			}
		}
		char UA[] = {FLAG, ADD1, CUA, ADD1 ^ CUA, FLAG};
		write(fd, UA, 5);
		return 1;
	}
	return -1;
}

int llwrite(char *buf, int bufSize)
{
	char inp[MAX_PAYLOAD_SIZE + 7];
	char bcc2;
	char stuff[(MAX_PAYLOAD_SIZE * 2) + 4]; //Worst Case Scenario

	send = 1;
	alarmEnabled = FALSE;
	alarmCount = 0;

	inp[0] = FLAG;
	inp[1] = ADD1;

	if (prevAckN == ACK1)
	{
		inp[2] = SEQ1;
	}
	else if (prevAckN == ACK2)
	{
		inp[2] = SEQ2;
	}
	else
	{
		return -1;
	}

	inp[3] = inp[1] ^ inp[2];

	int j = 4;

	for (int i = 0; i < bufSize; i++)
	{ // Passa bytes de dados do buffer para input
		if (i == 0)
			bcc2 = buf[i];
		else
			bcc2 = bcc2 ^ buf[i]; // Data blockcheck
		inp[j] = buf[i];
		j++;
	}

	inp[j] = bcc2; // Data BlockCheck and final flag
	inp[j + 1] = FLAG;

	int stuffed_frame_size = stuff_byte(inp, stuff, j + 2); //Stuff input

	while (1)
	{
		if (send == 1)
		{
			write(fd, &stuff, stuffed_frame_size);
			send = 0;
		}

		if (alarmEnabled == FALSE)
		{
			alarm(ll.timeOut); // Set alarm to be triggered in ll.timeOut seconds
			alarmEnabled = TRUE;
		}

		input = read(fd, &currChar, 5);
		alarm(0);

		if (input == 0)
		{ //Error: No characters read
			send = 1;
			continue;
		}

		if (currChar[2] == REJ1 || currChar[2] == REJ2)
		{ //If it's a REJECT packet, immediately resend
			send = 1;
			continue;
		}

		if (stuff[2] == SEQ1)
		{ //Sequence number check
			if (currChar[2] == ACK2)
			{
				prevSeqN = prevSeqN ^ 0x02;
				prevAckN = currChar[2];
				break;
			}
			else if (currChar[2] != (ACK2))
			{
				send = 1; // Reenvia
			}
		}

		else if (stuff[2] == SEQ2)
		{

			if (currChar[2] == ACK1)
			{
				prevSeqN = prevSeqN ^ 0x02;
				prevAckN = currChar[2];
				break;
			}
			else if (currChar[2] != (ACK1))
			{
				send = 1; //Reenvia
			}
		}
	}

	return bufSize;
}

int llread(char *packet)
{
	char incoming[MAX_PAYLOAD_SIZE * 2];
	char bcc2;
	int destuffsize;
	char destuffed[MAX_PAYLOAD_SIZE];
	int bytes;
	int m = 0;

	while (1)
	{
		bytes = read(fd, &incoming, MAX_PAYLOAD_SIZE * 2);
		if (bytes > 0)
			break;
	}

	if (incoming[0] != FLAG)
		return -1; // No start flag
	if ((incoming[1] ^ incoming[2]) != incoming[3])
		return -1;											//Error in Header
	destuffsize = destuff_byte(incoming, destuffed, bytes); // Destuffing

	for (int i = 4; i < destuffsize - 2; i++)
	{ // BCC2 check
		if (i == 4)
			bcc2 = destuffed[i];
		else
			bcc2 = bcc2 ^ destuffed[i];
	}

	if (bcc2 != incoming[bytes - 2])
	{ //Error in Data
		if (prevAckN != incoming[2])
		{
			TotalRejects++;
			char REJ[] = {FLAG, ADD1, prevAckN ^ 0x04, ADD1 ^ prevAckN ^ 0x04, FLAG};
			write(fd, REJ, 5);
			return 0;
		}
		else
		{
			char RR[] = {FLAG, ADD1, prevAckN, ADD1 ^ prevAckN ^ 0x20 ^ 0x01, FLAG};
			write(fd, RR, 5);
			return 0;
		}
	}

	else
	{ //SEQUENCE Number check

		char sequenceTest;
		if (prevAckN == ACK1)
		{
			sequenceTest = SEQ1;
		}
		else
		{
			sequenceTest = SEQ2;
		}
		fflush(stdout);
		if (sequenceTest == incoming[2])
		{
			prevAckN = prevAckN ^ 0x20;
			prevSeqN = incoming[2];
			char RR[] = {FLAG, ADD1, prevAckN, ADD1 ^ prevAckN, FLAG};
			write(fd, RR, 5);
		}
		else
		{
			char RR[] = {FLAG, ADD1, prevAckN, ADD1 ^ prevAckN ^ 0x20, FLAG};
			write(fd, RR, 5);
			return 0;
		}
	}

	for (int j = 4; j < destuffsize - 2; j++)
	{ // Data/payload to be send to file

		packet[m] = destuffed[j];
		m++;
	}

	return m;
}

int llclose(int showStatistics)
{

	if (ll.role == 0)
	{ //If TRANSMITTER
		if (showStatistics == 1)
		{
			printf("Number of Retransmissions: %d \n", TotalAlarms);
		}
		while (1)
		{

			char DISC[] = {FLAG, ADD1, CDISC, ADD1 ^ CDISC, FLAG};
			if (send == 1)
			{
				write(fd, DISC, 5);
				send = 0;
			}

			if (alarmEnabled == FALSE)
			{
				alarm(ll.timeOut); // Set alarm to be triggered in ll.timeOut seconds
				alarmEnabled = TRUE;
			}

			input = read(fd, &currChar, 5);
			alarm(0);
			if ((currChar[1] ^ currChar[2]) != (ADD2 ^ CDISC))
				send = 1;
			if (send == 0)
				break;
		}

		char UA[] = {FLAG, ADD2, CUA, ADD2 ^ CUA, FLAG};
		write(fd, UA, 5);

		// Restore the old port settings
		if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
		{
			perror("tcsetattr");
			return -1;
		}

		close(fd);
		printf("Connection closed\n");
		return 1;
	}

	if (ll.role == 1)
	{ //If RECEIVER
		if (showStatistics == 1)
		{
			printf("Number of Rejects: %d \n", TotalRejects);
		}
		enum flag_state
		{
			START,
			FLAG_GOT,
			A_GOT,
			C_GOT,
			BCC_GOT,
			STOP
		};
		enum flag_state flag = START;

		while (flag != STOP)
		{
			input = read(fd, &singleChar, 1);

			switch (singleChar)
			{
			case FLAG:
				if (flag == BCC_GOT)
					flag = STOP;
				else
					flag = FLAG_GOT;
				break;
			case ADD1:
				if (flag == FLAG_GOT)
					flag = A_GOT;
				else
					flag = START;
				break;
			case CDISC:
				if (flag == A_GOT)
					flag = C_GOT;
				else
					flag = START;
				break;
			case ADD1 ^ CDISC:
				if (flag == C_GOT)
					flag = BCC_GOT;
				else
					flag = START;
				break;
			default:
				flag = START;
				break;
			}
		}
		char DISC[] = {FLAG, ADD2, CDISC, ADD2 ^ CDISC, FLAG};
		write(fd, DISC, 5);
		printf("Connection closed\n");
		return 1;
	}
	return -1;
}
