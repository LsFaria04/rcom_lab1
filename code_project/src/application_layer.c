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

    unsigned char *buf = (unsigned char*)malloc(sizeof(unsigned char) * 3);

    buf[0] = 0x90;
    buf[1] = 0x20;
    buf[2] = 0x7e;

    //llwrite(buf, 3);

    llclose(TRUE);

    free(buf);
    
}
