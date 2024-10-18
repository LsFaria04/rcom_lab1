// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries; 
    connectionParameters.role = strcmp(role, "tx") ? LlRx : LlTx;
    strcpy(connectionParameters.serialPort,serialPort);
    connectionParameters.timeout = timeout;

    llopen(connectionParameters);

    if(connectionParameters.role == LlTx){
        unsigned char *buf = (unsigned char*)malloc(sizeof(unsigned char) * 5);

        /*buf[0] = 0x7E;
        buf[1]=0X03;
        buf[2] = 0x0B;
        buf[3]=0x03^0x0B;
        buf[4] = 0x7E;

        int res=llwrite(buf, 5);
        printf("aa\n");
        printf("%d\n",res);
        */
        free(buf);
        llclose(FALSE);
    }

    else{
        unsigned char *packet = (unsigned char*)malloc(sizeof(unsigned char) * MAX_PAYLOAD_SIZE);
        llread(packet);

        for(int i = 0; i < 5; i++){
            printf("char n%d = %x\n", i, packet[i]);
        }

        free(packet);
    }

    llclose(TRUE); 
    
}
