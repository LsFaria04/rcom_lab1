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

static int frame_numb = 0; //auxiliary varible to count the frames received

static int bcc2_control = 0; //used to check if the bcc2 control is correct

static LinkLayerRole role; //used to check the role in the connection

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
void state_machine_UA(unsigned char *received_buf, bool isSender){
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
            if((*received_buf == A_RECEIVER && !isSender) || (*received_buf == A_SENDER && isSender) ){
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
            if(*received_buf == (C_UA ^ A_RECEIVER) && !isSender ){
                state_command = BCC1_OK;
            }
            else if (*received_buf == (C_UA ^ A_SENDER) && isSender){
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
            if(*received_buf == A_SENDER){
                state_command = A_RCV;
            }
            else if(*received_buf == FLAG){
                state_command = FLAG_RCV;
            }
            else{
                state_command = START;
            }
            break;

        case A_RCV:

            unsigned char rrByte = C_RR0 + ((frame_numb + 1) % 2);
            unsigned char rejByte = C_RR0 + (frame_numb % 2);

            if(*received_buf == rrByte){
                //reveived a rr byte
                control = rrByte;
                state_command = C_RCV;
            }
            else if(*received_buf == rejByte){
                //reveived a rej byte
                control = rejByte;
                state_command = C_RCV;
                *isRej = true;
            }
            else if(*received_buf == FLAG){
                state_command = FLAG_RCV;
            }
            else{
                state_command = START;
            }
            break;

        case C_RCV:
            if(*received_buf == (A_SENDER ^ control)){
                state_command = BCC1_OK;
            }  
            else if(*received_buf == FLAG){
                state_command = FLAG_RCV;
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

//state machine for receiving the disc frame
void state_machine_disc(unsigned char *byte, bool isSender){
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
            if((*byte == A_SENDER && isSender) || (*byte==A_RECEIVER && !isSender)){
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
            if(*byte == C_DISC){
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
            if(*byte == (A_SENDER ^ C_DISC) || *byte == (A_RECEIVER ^ C_DISC)){
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

void state_machine_data_frame(unsigned char *byte, bool *isRej, bool *isDisc){
    switch(state_frame){
        case START:
            if(*byte == FLAG){
                state_frame = FLAG_RCV;
            }
            else{
                *isRej = true;
                state_frame = START;
            }
            break;

        case FLAG_RCV:
            if(*byte == A_SENDER){
                state_frame = A_RCV;
            }
            else if(*byte == FLAG){
                state_frame = FLAG;
                }
            else{
                state_frame = START;
            }
            break;

        case A_RCV:
            if(*byte == N(frame_numb)){
                state_frame = C_RCV;
            }
            else if(*byte == FLAG){
                state_frame = FLAG;
                }
            else if(*byte == C_DISC){
                //the program is going to terminate
                *isDisc=true;
                state_frame=C_RCV;
            }
            else{
                *isRej = true;
                state_frame = START;
            }
            break;

        case C_RCV:
            if(*byte == (A_SENDER ^ N(frame_numb))){
                state_frame = BCC1_OK;
            } 
            else if(*byte==(A_SENDER ^ C_DISC)) {
                
                state_frame=BCC1_OK;
            }
            else if(*byte == FLAG){
                state_frame= FLAG;
                }
            else
            {
                *isRej = true;
                state_frame = START;
            }
            break;

        case BCC1_OK:
            if(*byte==FLAG && *isDisc){
                
                state_frame=END;
            }
            else if(*byte == FLAG){
                state_frame = FLAG;
            }    
            else{
                state_frame = DATA;
            }
            break;

        case DATA:
            if(*byte == bcc2_control){
                state_frame = BCC2_OK;
            }
            break;
        
        case BCC2_OK:
            if(*byte == FLAG){
                state_frame = END;
            }
            else{
                *isRej = true;
                state_frame = START;
            }
            break;

        default:
            state_frame = START;
            break;
        } 

}

//creates a frame and returns it's size
int createDataFrame(unsigned char *frame, const unsigned char *buf, int bufSize){
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

//Creates a unumbered frame
void createUnnumberedFrame(unsigned char *frame, command cmd, bool isSender){
    frame[0]=FLAG;
    frame[4] = FLAG;

    if(isSender){
        frame[1]=A_SENDER;
    }
    else{
        frame[1]=A_RECEIVER;
    }

    switch(cmd){
        case UA:
            frame[2] = C_UA;
            frame[3] = frame[1] ^ C_UA;
            break;
        case DISC:
            frame[2] = C_DISC;
            frame[3] = frame[1] ^ C_DISC;
            break;
        case SET:
            frame[2] = C_SET;
            frame[3] = frame[1] ^ C_SET;
            break;
    }
    
}


void createSuperVisionFrame(unsigned char *frame, bool *isRej){
    frame[0] = FLAG;
    frame[1] = A_SENDER;

    if(*isRej){
        frame[2] = C_REJ + (frame_numb % 2);
        frame[3] = A_SENDER ^ (C_REJ + (frame_numb % 2));
    }
    else{
        frame[2] = C_RR0 + ((frame_numb + 1) % 2);
        frame[3] = A_SENDER ^ (C_RR0 + ((frame_numb + 1) % 2));
    }

    frame[4] = FLAG;
}

//sends an unnumbered frame (command) 
int sendUnnumberedFrame(command cmd, bool isSender){
    unsigned char *frame = (unsigned char*)malloc(sizeof(unsigned char) * 5);

    //creates the frame to send
    createUnnumberedFrame(frame, cmd, isSender);
   
    //sends the command 
    int bytes = writeBytesSerialPort(frame, 5);
    printf("%d bytes have been written\n",bytes);
    
    //waits until all bytes have been written in the serial port
    sleep(1);
    

    if(bytes < 0){
        printf("failed do send\n");
        return -1;
    }

    return 0;
}

//receives and unnumbered frame (command)
int receiveUnnumberedFrame(command cmd, bool isSender, bool hasTimeout){
    state_command = START;
    unsigned char *received_frame = (unsigned char*)malloc(sizeof(unsigned char));


    int byte = 0;
    state_command = START;

    while (true)
    {   
        
        if((alarmEnabled == FALSE) && hasTimeout){
            //alarm has reached is timeout
            return 1;
        }
        
        
        // Returns after 1 char have been input
        byte = readByteSerialPort(received_frame);

        if(byte == -1){
            break;
        }
        else if(byte == 0){
            continue;
        }

        //choose the correct state machine
        switch(cmd){
            case UA:
                state_machine_UA(received_frame, isSender);
                break;
            case DISC:
                state_machine_disc(received_frame, isSender);
                break;
            case SET:
                state_machine_set(received_frame);
                break;

        }



        if(state_command == END){
            printf("Command %d reveived successfully\n", cmd);
            
            if(hasTimeout){
                alarm(0);
            }

            free(received_frame);
            return 0;
        }

    }

    if(hasTimeout){
        alarm(0);
    }

    free(received_frame);
    return -1;


}

//sends a supervision frame
int sendSupervisionFrame(bool *isRej){
    unsigned char *frame = (unsigned char*)malloc(sizeof(unsigned char) * 5);
    createSuperVisionFrame(frame, isRej);

    //sends the supervision frame
    int bytes = writeBytesSerialPort(frame, 5);

    // Wait until all bytes have been written to the serial port
    sleep(1);

    if(bytes < 0){
        return -1;
    }
    free(frame);

    return 0;
}

int connectToReceiver(const unsigned char *frame){

    printf("New termios structure set\n");

    //starts the alarm
    (void)signal(SIGALRM, alarmHandler);

    
    int byte = 0;
    alarmCount = 0;
    alarmEnabled = FALSE;
    state_command = START;



    //waits timeout time for the UA message. Tries n times to send the message
    while (alarmCount < nRetransmissions)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(timeout); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
            
            sendUnnumberedFrame(SET, true);

        }

        byte = receiveUnnumberedFrame(UA, true, true);

        if(byte == 0){
            printf("Connection to receiver completed\n");
            alarm(0);
            return 0;
        }

        else if(byte < 0){
            break;
        }
        

    }

    alarm(0);
    return -1;
}

//receives the set frame and sends the UA frame in order to connect to the receiver
int connectToSender(){

    if(receiveUnnumberedFrame(SET, false, false) < 0 ){
        return -1;
    }

    if(sendUnnumberedFrame(UA, true) < 0){
        return -1;
    }

    printf("Connection to the receiver completed\n");

    return 0;
}

//terminates the connection between the receiver and the transmitter
int terminate_connection(){
    
    //starts the alarm
    (void)signal(SIGALRM, alarmHandler);

    
    int byte = 0;
    alarmCount = 0;
    alarmEnabled = FALSE;
    state_command = START;



    //waits timeout time for the UA message. Tries n times to send the message
    while (alarmCount < nRetransmissions)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(timeout); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
            
            if(sendUnnumberedFrame(DISC, true) != 0){
                return -1;
            }

        }

        byte = receiveUnnumberedFrame(DISC, false, true);


        if(byte == 0){
            if(sendUnnumberedFrame(UA, false) < 0){
                return -1;
            }
            alarm(0);
            return 0;
        }

        else if(byte < 0){
            break;
        }
        

    }

    alarm(0);
    return -1;
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

    role = connectionParameters.role;
    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    //Frame used for the Set and UA frames
    unsigned char *frame = (unsigned char*)malloc(sizeof(unsigned char) * 5);

    switch(connectionParameters.role){
        case LlTx:

            if(connectToReceiver(frame) < 0){
                printf("Timeout when sending the set frame\n");
                return -1;
            }
            break;

        case LlRx:

            if(connectToSender() < 0){
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
    alarmEnabled = FALSE;

    //create the frame to send
    unsigned char *frame = (unsigned char*)malloc(sizeof(unsigned char) * (bufSize * 2 + 6)); //allocate space for the worst case scenario
    int frame_size = createDataFrame(frame, buf, bufSize);

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
                

                //sends the frame
                bytes = writeBytesSerialPort(frame, frame_size);
                printf("%d bytes written\n", bytes);
                // Wait until all bytes have been written to the serial port
                sleep(1);

            }
            
            // Returns after 1 char have been input
            readByteSerialPort(received_frame);
            
            //change state depending on the byte received
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
        if(!isRej){
            //The transmitter isn't trying to send again because is a timeout
            break;
        }
        isRej = false;
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
    int byte_count = 0;
    //allocate space for the received frame buffer
    unsigned char *received_frame = (unsigned char*)malloc(sizeof(unsigned char));

    bool isRej = false;
    bool isSpecial = false;
    bool isDisc = false;
    int byte = 0;
    state_frame = START;

    //only exits after completing the data receiving
    while(true){
            
            byte = readByteSerialPort(received_frame);
            

            if(byte < 0){
                printf("something went wrong when reading the data bytes\n");
                break;
            }

            //didn't receive any new byte
            if(byte == 0){
                printf("no byte\n");
                continue;
            }
            
            state_machine_data_frame(received_frame, &isRej, &isDisc);



            if(isRej){
                printf("rej\n");
                sendSupervisionFrame(&isRej);
                isRej = false;
                continue;
            }

            if(state_frame == A_RCV && *received_frame == C_DISC){

                isDisc = true;
                printf("Receiving disc\n");
            }
            

            //handles the special bytes using the byte stuffing mechanism
            if(isSpecial){
                if(*received_frame == (FLAG ^ 0x20)){
                    packet[byte_count] = FLAG;
                    bcc2_control ^= FLAG;
                }
                else{
                    packet[byte_count] = ESC;
                    bcc2_control ^= ESC;
                }

                isSpecial = false;
                byte_count++;
            }

            //checks if the byte received is data
            if((state_frame == DATA)){

                

                //checks if the byte receivd is a special character from the byte stuffing mechanism
                if(*received_frame == ESC){
                    isSpecial = true;
                }
                else{
                    packet[byte_count] = *received_frame;
                    byte_count++;
                    bcc2_control ^= *received_frame;
                }
                
            }

            //if the frame is successfully received, send the rr to confirm an return the number of chars read or a disc frame
            if(state_frame == END){
                if(isDisc){
                    //sends the disc frame to the sender and receive an UA 
                    sendUnnumberedFrame(DISC, false);
                    receiveUnnumberedFrame(UA, false, false);

                    free(received_frame);
                    return 0;
                    
                }
                else{
                    printf("Frame %d received successfully\n", frame_numb);
                    sendSupervisionFrame(&isRej);
                    return byte_count + 1;
                }
                

        }  
        
    }

    free(received_frame);
    return -1;
}


////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{   
    if(role == LlTx){
        if(terminate_connection() < 0){
            return -1;
        }
    }

    if(showStatistics){
        printf("Number of timeouts = %d\n", number_timeouts);
        printf("Number of retransmissions = %d\n", number_rTransmissions);
    }

    int clstat = closeSerialPort();
    return clstat;
}
