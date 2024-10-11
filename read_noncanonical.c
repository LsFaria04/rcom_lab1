// Read from serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include "msg_bytes.h"
#include "states.h"

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 5

volatile int STOP = FALSE;
static int state = Start;
unsigned char *byte;

//checks if the message received is the set frame
bool isSet (unsigned char *buf){
    return buf[0] == FLAG && buf[1] == A_SENDER && buf[2] == C_SET && buf[3] == (A_SENDER ^ C_SET) && buf[4] == FLAG; 
}

//state machine
void state_machine(){
    switch (state)
       {

        case Start:
            if(*byte == FLAG){
                state = FLAG_RCV;
            }
            else{
                state = Start;
            }
            break;

        case FLAG_RCV:
            if(*byte == A_SENDER){
                state = A_RCV;
            }
            else if(*byte == FLAG){state = FLAG;}
            else{
                state = Start;
            }
            break;

        case A_RCV:
            if(*byte == C_SET){
                state = C_RCV;
            }
            else if(*byte == FLAG){state=FLAG;}
            else{
                state = Start;
            }
            break;

        case C_RCV:
            if(*byte == (A_SENDER ^ C_SET)){
                state = BCC_OK;
            }  
            else if(*byte == FLAG){state=FLAG;}
            else
            {
                state = Start;
            }
            break;

        case BCC_OK:
            if(*byte == FLAG){
                state = END;
            }    
            else{
                state = Start;
            }
            break;

        default:
            state = Start;
            break;
        }
}


int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 1;  // Blocking read until 1 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
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
    int bytes = 0;

    //allocate a memory buffer to receive a byte from the transmitter
    byte = (unsigned char*)malloc(sizeof(unsigned char));
    state = Start;  //first state is start
    while (STOP == FALSE)
    {
        *byte = 0; 
        bytes = read(fd, byte, 1); //read a byte from the serial port
        state_machine(); //run the state machine with the new byte

        if(state == END){

            printf("Set frame received successfully\n");
            STOP = TRUE;
       }

    }

    //The UA frame message is created and send to the sender
    //insert into the buffer

    unsigned char buf[5] = {0};
    buf[0] = FLAG;
    buf[1] = A_RECEIVER;
    buf[2] = C_UA;
    buf[3] = A_RECEIVER ^ C_UA;
    buf[4] = FLAG;

    //sends the bytes
    bytes = write(fd, buf, BUF_SIZE);
    sleep(1);
    printf("%d bytes written\n", bytes);

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }
    
    free(byte);
    close(fd);

    return 0;
}
