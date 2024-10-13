// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

//alarm help variables
static int alarmEnabled = FALSE;
static int alarmCount = 0;

//states when receiving commands and frames
state state_command = START;
state state_frame = START;

//Values used for the alarm and retransmission system
int nRetransmissions = 0;
int timeout = 0;

static int fd = -1;

static int control = 0; //auxiliary variable to store the control value reveived in a frame

static int frame_numb = 0;

//used for the statistics
static int number_timeouts = 0;
static int number_rTransmissions = 0;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

//state machine for receiving the UA frame
void state_machine_UA(unsigned char *received_buf){
    switch (state_command)
       {

        case START:
            if(*received_buf == FLAG){
                state_command = FLAG_RCV;
            }
            else{
                state_command = START;
            }
            break;

        case FLAG_RCV:
            if(*received_buf == A_RECEIVER){
                state_command = A_RCV;
            }
            else if(*received_buf == FLAG){
                state_command = FLAG;
            }
            else{
                state_command = START;
            }
            break;

        case A_RCV:
            if(*received_buf == C_UA){
                state_command = C_RCV;
            }
            else if(*received_buf == FLAG){
                state_command = FLAG;
            }
            else{
                state_command = START;
            }
            break;

        case C_RCV:
            if(*received_buf == (A_RECEIVER ^ C_UA)){
                state_command = BCC1_OK;
            }  
            else if(*received_buf == FLAG){
                state_command = FLAG;
            }
            else
            {
                state_command = START;
            }
            break;

        case BCC1_OK:
            if(*received_buf == FLAG){
               state_command = END;
            }    
            else{
                state_command = START;
            }
            break;

        default:
            state_command = START;
            break;
        }
}

void state_machine_RR_REJ(unsigned char *received_buf, bool *isRej){
        switch (state_command)
       {

        case START:
            if(*received_buf == FLAG){
                state_command = FLAG_RCV;
            }
            else{
                state_command = START;
            }
            break;

        case FLAG_RCV:
            if(*received_buf == A_RECEIVER){
                state_command = A_RCV;
            }
            else if(*received_buf == FLAG){
                state_command = FLAG;
            }
            else{
                state_command = START;
            }
            break;

        case A_RCV:
            if(*received_buf == C_RR0 + (frame_numb % 2) + 1){
                control = C_RR0 + (frame_numb % 2) + 1;
                state_command = C_RCV;
            }
            else if(*received_buf == C_RR0 + (frame_numb % 2)){
                control = C_REJ + (frame_numb % 2);
                state_command = C_RCV;
                *isRej = true;
            }
            else if(*received_buf == FLAG){
                state_command = FLAG;
            }
            else{
                state_command = START;
            }
            break;

        case C_RCV:
            if(*received_buf == (A_RECEIVER ^ control)){
                state_command = BCC1_OK;
            }  
            else if(*received_buf == FLAG){
                state_command = FLAG;
            }
            else
            {
                state_command = START;
            }
            break;

        case BCC1_OK:
            if(*received_buf == FLAG){
                state_command = END;
            }    
            else{
                state_command = START;
            }
            break;

        default:
            state_command = START;
            break;
        }
}

//state machine for receiving the set frame
void state_machine_set(unsigned char *byte){
    switch (state_command)
       {

        case START:
            if(*byte == FLAG){
                state_command = FLAG_RCV;
            }
            else{
                state_command = START;
            }
            break;

        case FLAG_RCV:
            if(*byte == A_SENDER){
                state_command = A_RCV;
            }
            else if(*byte == FLAG){
                state_command = FLAG;
                }
            else{
                state_command = START;
            }
            break;

        case A_RCV:
            if(*byte == C_SET){
                state_command = C_RCV;
            }
            else if(*byte == FLAG){
                state_command = FLAG;
                }
            else{
                state_command = START;
            }
            break;

        case C_RCV:
            if(*byte == (A_SENDER ^ C_SET)){
                state_command = BCC1_OK;
            }  
            else if(*byte == FLAG){
                state_command = FLAG;
                }
            else
            {
                state_command = START;
            }
            break;

        case BCC1_OK:
            if(*byte == FLAG){
                state_command = END;
            }    
            else{
                state_command = START;
            }
            break;

        default:
            state_command = START;
            break;
        }
}

//creates the set frame
void createSetFrame(unsigned char *frame){
    frame[0] = FLAG;
    frame[1] = A_SENDER;
    frame[2] = C_SET;
    frame[3] = A_SENDER ^ C_SET;
    frame[4] = FLAG;
}

//Creates the UA frame
void createUAFrame(unsigned char *frame){
    frame[0] = FLAG;
    frame[1] = A_RECEIVER;
    frame[2] = C_UA;
    frame[3] = A_RECEIVER ^ C_UA;
    frame[4] = FLAG;
}

//creates a frame and returns it's size
int create_frame(unsigned char *frame, const unsigned char *buf, int bufSize){
    frame[0] = FLAG;
    frame[1] = A_SENDER;
    frame[2] = N(frame_numb);
    frame[3] = frame[1] ^ frame[2];

    //inserts the data into the frame
    unsigned char bcc2 = 0;
    int frame_index = 4;
    for(int i = 0; i < bufSize; i++){
        
        if(buf[i] == FLAG){
            frame[frame_index] = ESC;
            frame[frame_index + 1] = FLAG ^ 0x20;
            frame_index = frame_index + 2;
        }
        else if(buf[i] == ESC){
            frame[frame_index] = ESC;
            frame[frame_index + 1] = ESC ^ 0x20;
            frame_index = frame_index + 2;
        }
        else{
            frame[frame_index] = buf[i];
            frame_index++;
        }

        bcc2 ^= buf[i];
    }

    frame[frame_index] = bcc2;
    frame_index++;
    frame[frame_index] = FLAG;

    return frame_index + 1;
}

int sendSetFrame(const unsigned char *frame){

    printf("New termios structure set\n");

    //starts the alarm
    (void)signal(SIGALRM, alarmHandler);

    int byte = 0;
    alarmCount = 0;
    state_command = START;

    //allocate space for the received frame buffer
    unsigned char *received_frame = (unsigned char*)malloc(sizeof(unsigned char));


    //waits 3s for the UA message. Tries 3 times to send the message
    while (alarmCount < nRetransmissions)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(timeout); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
            

            //sends the set message
            byte = writeBytesSerialPort(frame, 5);
            printf("%d bytes written\n", byte);
            // Wait until all bytes have been written to the serial port
            sleep(1);

        }
        
        // Returns after 1 char have been input
        byte = readByteSerialPort(received_frame);

        if(byte == -1){
            return -1;
        }
        else if(byte == 0){
            continue;
        }

        //state machine for receiving the UA frame
        state_machine_UA(received_frame);

        if(state_command == END){
            printf("UA reveived successfully\n");
            free(received_frame);
            return 0;
        }

    }

    free(received_frame);
    alarm(0);
    return -1;
}

//receives the set frame and sends the UA frame
int sendUAFrame(const unsigned char *frame){
    printf("New termios structure set\n");

    int byte = 0;

    //allocate space for the received frame buffer
    unsigned char *received_frame = (unsigned char*)malloc(sizeof(unsigned char));

    while (true)
    {
        //sends the set message
        byte = readByteSerialPort(received_frame);

        if(byte == -1){
            return -1;
        }

        state_machine_set(received_frame);

        if(state_command == END){
            printf("Received the set frame successfully\n");
            break;
        }

    }

    // Returns after 1 char have been input
    byte = writeBytesSerialPort(frame, 5);
    printf("%d bytes written\n", byte);
    // Wait until all bytes have been written to the serial port
    sleep(1);

    free(received_frame);
    return 0;
}
////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    fd = openSerialPort(connectionParameters.serialPort,connectionParameters.baudRate);
    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        return -1;
    }

    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    //Frame used for the Set and UA frames
    unsigned char *frame = (unsigned char*)malloc(sizeof(unsigned char) * 5);

    switch(connectionParameters.role){
        case LlTx: 
            createSetFrame(frame);

            //send the set frame
            if(sendSetFrame(frame) < 0){
                printf("Timeout when sending the set frame\n");
                return -1;
            }
            break;
        case LlRx:
            createUAFrame(frame);

            //send the UA frame after receiveing the set frame
            if(sendUAFrame(frame) < 0){
                printf("Something went wrong when receiving the set frame\n");
                return -1;
            }
            break;
        default:
            printf("The role isn't available\n");
            return -1;

    }
    
    free(frame);
    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    //checks if the SIZE of maximum acceptable payload is exceeded
    if(bufSize > MAX_PAYLOAD_SIZE){
        return -1;
    }

    //starts the alarm
    (void)signal(SIGALRM, alarmHandler);

    alarmCount = 0;

    //create the frame to send
    unsigned char *frame = (unsigned char*)malloc(sizeof(unsigned char) * (bufSize * 2 + 6)); //allocate space for the worst case scenario
    int frame_size = create_frame(frame, buf, bufSize);

    //allocate space for the received frame buffer
    unsigned char *received_frame = (unsigned char*)malloc(sizeof(unsigned char));


    bool isRej = false;
    int bytes = 0;
    state_command = START;

    //only exits after completing the data sending
    while(true){

        //waits 3s for the UA message. Tries 3 times to send the message
        while (alarmCount < nRetransmissions)
        {
            if (alarmEnabled == FALSE)
            {
                alarm(timeout); // Set alarm to be triggered in 3s
                alarmEnabled = TRUE;
                

                //sends the frmame
                bytes = writeBytesSerialPort(frame, frame_size);
                printf("%d bytes written\n", bytes);
                // Wait until all bytes have been written to the serial port
                sleep(1);

            }
            
            // Returns after 1 char have been input
            readByteSerialPort(received_frame);
            
            state_machine_RR_REJ(received_frame, &isRej);

            //if the frame received is rej, exit the loop and try again. Else, exit the function and increase the frame counter
            if(state_command == END){
                if(isRej){
                    printf("Frame %d was sent with problems. Trying again\n", frame_numb);
                    break;
                }
                else{
                    printf("Frame %d sent successfully\n", frame_numb);
                    frame_numb++;
                    free(received_frame);
                    return bytes;
                }  
            }

            
        }
        alarm(0);//reset the alarm
        
    }
    free(received_frame);
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{   
    if(showStatistics){
        printf("Number of timeouts = %d\n", number_timeouts);
        printf("Number of retransmissions = %d\n", number_rTransmissions);
    }

    int clstat = closeSerialPort();
    return clstat;
}
