// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


// Calculates log2 of number.  
double log2( double n )  
{  
    // log(n)/log(2) is log2.  
    return log( n ) / log( 2 );  
}

unsigned char * createControlPacket(int c, const char *filename, int file_size, int *packet_size){

    int numb_Bytes = (log2(file_size) / 8) + 1; //number of octets (bytes) used in V1
    unsigned char L1 = (unsigned char) numb_Bytes;

    unsigned char L2 = (unsigned char) strlen(filename);
    *packet_size = L2 + L1 + 5;

    unsigned char * packet = (unsigned char*)malloc(sizeof(unsigned char));

    packet[0] = c;
    packet[1] = 0; //first send the file size
    packet[2] = L1;

    int packet_idx = 3;

    for(unsigned char i = 0; i < L1; i++){
        packet[packet_idx++] = L1 & 0xFF;
        file_size >>= 8;
    }

    packet[packet_idx++] = 1; //file name
    packet[packet_idx++] = L2;
    memcpy(&packet[packet_idx], filename, L2);

    return packet;
}

void processControlPacket(unsigned char* packet,  char *filename, int *file_size, int packet_size){

    int L1 = packet[2];

    //reconstruct the file size
    for(unsigned char i = 0; i < L1; i++){
        *file_size |= packet[2 + i];

        if(i == L1 - 1){
            break;
        }

        *file_size <<= 8;  
    }

    //get the file name
    int L2 = packet[3 + L1 + 1];
    memcpy(filename, &packet[3 + L1 + 2], L2);
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries; 
    connectionParameters.role = strcmp(role, "tx") ? LlRx : LlTx;
    strcpy(connectionParameters.serialPort,serialPort);
    connectionParameters.timeout = timeout;

    if(llopen(connectionParameters) < 0){
        perror("Error in the connection\n");
        exit(-1);
    }

    switch(connectionParameters.role){
        case LlTx:

        FILE *file = fopen(filename, "r");
        if(file == NULL){
            perror("The file doesn't exist\n");
            exit(-1);
        }

        int file_size = (int) fseek(file, 0L, SEEK_END);
        rewind(file);

        //create the Start packet
        int packet_size = 0;
        unsigned char *packet = createControlPacket(1, filename, file_size, &packet_size);

        //send the start packet
        if(llwrite(packet, packet_size) < 0){
            printf("Error while writing the start packet\n");
            exit(-1);
        }

        free(packet);


        break;
        case LlRx:
            //create the packet
            unsigned char *packet_RC = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);

            //read the start packet from the serial port
            int packet_size_RC = -1;
            while ((packet_size_RC = llread(packet_RC)) < 0);

            //process the packet read
            int file_size_RC = 0;
            char *name = (char*)malloc(sizeof(unsigned char) * MAX_PAYLOAD_SIZE);
            processControlPacket(packet_RC, name, &file_size_RC, packet_size); 

            //llread(packet_RC);//just to receive the disk
            free(packet_RC);

        break;
        default:
            perror("Role does't exist\n");
            exit(-1);
    }
    
    if(connectionParameters.role == LlTx){
        unsigned char *buf = (unsigned char*)malloc(sizeof(unsigned char) * 5);

        buf[0] = 0x7E;
        buf[1]=  0X7E;
        buf[2] = 0x7D;
        buf[3]=  0x7D;
        buf[4] = 0x7E;

        llwrite(buf, 5);

        
        free(buf);
    }

    else{
        unsigned char *packet = (unsigned char*)malloc(sizeof(unsigned char) * MAX_PAYLOAD_SIZE);
        llread(packet);

        for(int i = 0; i < 5; i++){
            printf("char n%d = %x\n", i, packet[i]);
        }


        llread(packet);

        free(packet);
    }

    llclose(TRUE); 
    
}
