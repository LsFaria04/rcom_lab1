// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//Creates a control packet to send
unsigned char * createControlPacket(int c, const char *filename, int file_size, int *packet_size){

    int numb_Bytes = (log2(file_size) / 8) + 1; //number of octets (bytes) used in V1
    unsigned char L1 = (unsigned char) numb_Bytes;

    unsigned char L2 = (unsigned char) strlen(filename);
    *packet_size = L2 + L1 + 5;

    unsigned char * packet = (unsigned char*)malloc(sizeof(unsigned char) * *packet_size);

    packet[0] = c;
    packet[1] = 0; //first send the file size
    packet[2] = L1;

    int packet_idx = 3;

    for(unsigned char i = 0; i < L1; i++){
        packet[packet_idx++] = file_size & 0xFF;
       
        file_size >>= 8;
    }


    packet[packet_idx++] = 1; //file name
    packet[packet_idx++] = L2;
    memcpy(&packet[packet_idx], filename, L2);

    packet[*packet_size] = '\0';

    return packet;
}

//creates a data packet to send
unsigned char * createDataPacket(int seq, unsigned char *data, int data_size, int *packet_size){
    *packet_size = 4 + data_size; //four first bytes plus the data bytes

    unsigned char *packet = (unsigned char*)malloc(sizeof(unsigned char) * *packet_size);

    packet[0] = 2; //sending data;
    packet[1] = (unsigned int) seq % 100;
    
    // data_size = (256 * L2 + L1)
    unsigned char L2 = data_size >> 8 & 0xFF;
    unsigned char L1 = data_size & 0xFF;
    packet[2] = L2;
    packet[3] = L1;

    //copy the data to the packet
    memcpy(&packet[4], data, data_size );

    packet[*packet_size] = '\0';

    return packet;
}

//process a data packet by getting the needed info from it
void processDataPacket(unsigned char *packet, int *seq, unsigned char *data, int *data_size){
    //sequence byte
    *seq = packet[1];

    //get the size of the data
    unsigned char L2 = packet[2];
    unsigned char L1 = packet[3];
    *data_size = (L2 << 8) | L1;

    //copy the data from the packet to the received data buffer
    memcpy(data, &packet[4], *data_size);

    packet[*data_size + 4] = '\0';

}

//process a control packet by getting the needed info from it
void processControlPacket(unsigned char* packet,  char *filename, int *file_size, int packet_size){

    int L1 = packet[2];

    //reconstruct the file size
    for(unsigned char i = 0; i < L1; i++){
        *file_size |= packet[2 + L1 - i];   //inserts in reverse order

        
        if(i == L1 - 1){
            break;
        }

        *file_size <<= 8;  
    }

    //get the file name
    int L2 = packet[3 + L1 + 1];
    memcpy(filename, &packet[3 + L1 + 2], L2);

    packet[packet_size] = '\0';
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

        fseek(file, 0L, SEEK_END);
        int file_size = (int) ftell(file);
        rewind(file);


        //create the Start packet
        int packet_size = 0;
        unsigned char *controlPacket = createControlPacket(1, filename, file_size, &packet_size);

        //send the start packet
        if(llwrite(controlPacket, packet_size) < 0){
            printf("Error while writing the start packet\n");
            exit(-1);
        }

        free(controlPacket);

        int bytes_left = file_size;
        int sequence = 0;
        unsigned char *content = (unsigned char*)malloc(sizeof(unsigned char) * MAX_PAYLOAD_SIZE);
        //send data packets (1000 bytes at time)
        while(bytes_left > 0){
            int bytes_read = fread(content, sizeof(unsigned char) , MAX_PAYLOAD_SIZE - 4, file);

            unsigned char *packet = createDataPacket(sequence, content, bytes_read, &packet_size);

            if(llwrite(packet,packet_size) < 0){
                printf("Error while writing a data packet\n");
                exit(-1);
            }

            free(packet);

            bytes_left -= bytes_read;
            sequence++;
        }


        //create the end packet
        controlPacket = createControlPacket(3, filename, file_size, &packet_size);

        //send the start packet
        if(llwrite(controlPacket, packet_size) < 0){
            printf("Error while writing the end packet\n");
            exit(-1);
        }

        free(controlPacket);
        fclose(file);
        break;

        case LlRx:
            //create the packet
            unsigned char *packet_RC = (unsigned char *)malloc(sizeof(unsigned char) * MAX_PAYLOAD_SIZE);

            //read the start packet from the serial port
            int packet_size_RC = -1;
            while ((packet_size_RC = llread(packet_RC)) < 0);
            if(packet_size_RC < 0){
                printf("Error while reading the start packet\n");
                exit(-1);
            }
            if(packet_RC[0] != 1){
                printf("Expected Start control packet but received another\n");
                exit(-1);
            }

            //process the start packet
            int file_size_RC = 0;
            char *name = (char*)malloc(sizeof(unsigned char) * MAX_PAYLOAD_SIZE);
            processControlPacket(packet_RC, name, &file_size_RC, packet_size);

            //read the data
            int sequence_RC = 0;
            FILE* newFile = fopen(filename, "w+");
            unsigned char *content_received = (unsigned char*)malloc(sizeof(unsigned char) * MAX_PAYLOAD_SIZE);
            while(TRUE){
                while((packet_size_RC = llread(packet_RC)) < 0);
                if(packet_size_RC < 0){
                    printf("Error while reading a data packet\n");
                    exit(-1);
                }

                //checks if the packet received is a end control packet
                if(packet_RC[0] == 3){
                    char *name_end = (char*)malloc(sizeof(unsigned char) * MAX_PAYLOAD_SIZE);
                    int file_size_RC_end = 0;
                    processControlPacket(packet_RC, name_end,&file_size_RC_end, packet_size_RC);

                    if(strcmp(name, name_end) != 0){
                        printf("Name of the fil in the start packet is different from the one in the end packet\n");
                        exit(-1);
                    }

                    if(file_size_RC != file_size_RC_end){
                        printf("File lenght received at the Start packet is different from the one received at the end packet\n");
                        exit(-1);
                    }

                    free(name_end);

                    break;
                
                }
                else{
                    //data packet received. Write the data into the file
                    processDataPacket(packet_RC, &sequence_RC, content_received, &packet_size_RC);

                    fwrite(content_received, sizeof(unsigned char), packet_size_RC, newFile);
                }  
            }

            fclose(newFile);

            if(llread(packet_RC) < 0){
                 printf("Error while reading the end connection frames\n");
                exit(-1);
            }

            free(packet_RC);
            free(content_received);


        break;
        default:
            perror("Role does't exist\n");
            exit(-1);
    }

    llclose(TRUE); 
    
}
