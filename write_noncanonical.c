// Write to serial port in non-canonical mode
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
#include <signal.h>
#include <string.h>
#include "msg_bytes.h"
#include "link_layer.h"
#include "states.h"

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 5

static int state = Start;
volatile int STOP = FALSE;
static int alarmEnabled = FALSE;
static int alarmCount = 0;
static int fd = -1;
static unsigned char *received_buf;
static int frame_numb = 1;
static int control = 0;
static int nRetransmissions = 0;
static int timeout = 0;

struct termios oldtio;
struct termios newtio;

//checks if the message received is the UA frame
bool isUA (unsigned char *buf){
    return buf[0] == FLAG && buf[1] == A_RECEIVER && buf[2] == C_UA && buf[3] == (A_RECEIVER ^ C_UA) && buf[4] == FLAG; 
}

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

void createSetFrame(unsigned char *buf){
    buf[0] = FLAG;
    buf[1] = A_SENDER;
    buf[2] = C_SET;
    buf[3] = A_SENDER ^ C_SET;
    buf[4] = FLAG;
}

void state_machine_UA(){
    switch (state)
       {

        case Start:
            if(*received_buf == FLAG){
                state = FLAG_RCV;
            }
            else{
                state = Start;
            }
            break;

        case FLAG_RCV:
            if(*received_buf == A_RECEIVER){
                state = A_RCV;
            }
            else if(*received_buf == FLAG){state = FLAG;}
            else{
                state = Start;
            }
            break;

        case A_RCV:
            if(*received_buf == C_UA){
                state = C_RCV;
            }
            else if(*received_buf == FLAG){state=FLAG;}
            else{
                state = Start;
            }
            break;

        case C_RCV:
            if(*received_buf == (A_RECEIVER ^ C_UA)){
                state = BCC_OK;
            }  
            else if(*received_buf == FLAG){state=FLAG;}
            else
            {
                state = Start;
            }
            break;

        case BCC_OK:
            if(*received_buf == FLAG){
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

void state_machine_RR_REJ(bool *isRej){
        switch (state)
       {

        case Start:
            if(*received_buf == FLAG){
                state = FLAG_RCV;
            }
            else{
                state = Start;
            }
            break;

        case FLAG_RCV:
            if(*received_buf == A_RECEIVER){
                state = A_RCV;
            }
            else if(*received_buf == FLAG){
                state = FLAG;
            }
            else{
                state = Start;
            }
            break;

        case A_RCV:
            if(*received_buf == C_RR0 + frame_numb + 1){
                control = C_RR0 + frame_numb + 1;
                state = C_RCV;
            }
            else if(*received_buf == C_RR0 + frame_numb){
                control = C_REJ + frame_numb;
                state = C_RCV;
                *isRej = true;
            }
            else if(*received_buf == FLAG){
                state=FLAG;
            }
            else{
                state = Start;
            }
            break;

        case C_RCV:
            if(*received_buf == (A_RECEIVER ^ control)){
                state = BCC_OK;
            }  
            else if(*received_buf == FLAG){
                state=FLAG;
            }
            else
            {
                state = Start;
            }
            break;

        case BCC_OK:
            if(*received_buf == FLAG){
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

int sendSetFrame(const unsigned char *buf){

    printf("New termios structure set\n");

    //starts the alarm
    (void)signal(SIGALRM, alarmHandler);

    int bytes = 0;

    //waits 3s for the UA message. Tries 3 times to send the message
    while (alarmCount < nRetransmissions)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(timeout); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
            

            //sends the set message
            bytes = write(fd, buf, 5);
            printf("%d bytes written\n", bytes);
            // Wait until all bytes have been written to the serial port
            sleep(1);

        }
        received_buf = (unsigned char*)malloc(sizeof(unsigned char));
        *received_buf = 0;

        // Returns after 1 char have been input
        bytes = read(fd, received_buf, 1);
        
        state_machine_UA();

        if(state == END){
            printf("UA reveived successfully\n");
            free(received_buf);
            return 0;
        }

    }

    free(received_buf);
    alarm(0);
    return -1;
}

int llopen(LinkLayer connectionParameters){
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        return -1;
    }


    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 1 char received

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
        return -1;
    }

    //create the set frame
    unsigned char *buf = (unsigned char*)malloc(sizeof(unsigned char) * 5);
    createSetFrame(buf);

    //send the set frame
    if(sendSetFrame(buf) < 0){
        printf("Timeout when sending the set frame\n");
        return -1;
    }


    free(buf);
    return fd;

}


//sends the set frame to start the connection
int llwrite(const unsigned char *buf, int bufSize){

    //starts the alarm
    (void)signal(SIGALRM, alarmHandler);

    bool isRej = false;
    int bytes = 0;

    //only exits after completing the data sending
    while(true){

        //waits 3s for the UA message. Tries 3 times to send the message
        while (alarmCount < nRetransmissions)
        {
            if (alarmEnabled == FALSE)
            {
                alarm(timeout); // Set alarm to be triggered in 3s
                alarmEnabled = TRUE;
                

                //sends the set message
                bytes = write(fd, buf, bufSize);
                printf("%d bytes written\n", bytes);
                // Wait until all bytes have been written to the serial port
                sleep(1);

            }
            
            unsigned char *received_buf = (unsigned char*)malloc(sizeof(unsigned char));

            // Returns after 1 char have been input
            bytes = read(fd, received_buf, 1);
            
            state_machine_RR_REJ(&isRej);

            //if the frame received is rej, exit the loop and try again. Else, exit the function and increase the frame counter
            if(state == END){
                if(isRej){
                    printf("Frame %d was sent with problems. Trying again\n", frame_numb);
                    break;
                }
                else{
                    printf("Frame %d sent successfully\n", frame_numb);
                    frame_numb++;
                    free(received_buf);
                    return 0;
                }  
            }

            
        }
        alarm(0);//reset the alarm
        
    }
    free(received_buf);
    return -1;
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

    LinkLayer connectionParameters;
    connectionParameters.baudRate = BAUDRATE;
    connectionParameters.nRetransmissions = 3;
    connectionParameters.role = LlTx;
    strcpy(connectionParameters.serialPort,serialPortName);
    connectionParameters.timeout = 3;

    if(llopen(connectionParameters) < 0){
        return -1;
    }

/*
    unsigned char testBuf [6] = {0};
    testBuf[0] = FLAG;
    testBuf[1] = A_SENDER;
    testBuf[2] = 0;
    testBuf[3] = 0x10;
    testBuf[4] = A_SENDER ^ 0x10;
    testBuf[5] = FLAG

    if(llwrite(testBuf, 6) < 0){
        return -1;
    }
*/

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
