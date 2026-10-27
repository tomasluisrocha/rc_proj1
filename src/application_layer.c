// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

int buildControlPacket(unsigned char *packet, unsigned char controlType, long fileSize, const char *filename)
{
    int idx = 0;

    packet[idx++] = controlType;
    packet[idx++] = 0;

    unsigned char sizeBytes = sizeof(fileSize);
    packet[idx++] = sizeBytes;
    memcpy(&packet[idx], &fileSize, sizeBytes);
    idx += sizeBytes;

    packet[idx++] = 1;
    int nameLen = strlen(filename);
    packet[idx++] = (unsigned char)nameLen;
    memcpy(&packet[idx], filename, nameLen);
    idx += nameLen;

    return idx;
}

int parseControlPacket(const unsigned char *packet, int size, long *fileSize, char *name, int nameMax)
{
    if (packet == NULL || fileSize == NULL || name == NULL || size < 1 || nameMax <= 0)
        return -1;

    int idx = 1;
    while (idx + 2 <= size)
    {
        if (size - idx < 2)
            return -1;

        unsigned char type = packet[idx++];
        unsigned char length = packet[idx++];
        if (length > size - idx)
            return -1;

        if (type == 0)
        {
            if (length > sizeof(*fileSize))
                return -1;
            *fileSize = 0;
            memcpy(fileSize, &packet[idx], length);
        }
        else if (type == 1 && L < nameMax) 
        {
            memcpy(name, &packet[idx], L);
            name[L] = '\0';
        }
        idx += L;
    }

    return 0;
}

int buildDataPacket(unsigned char* packet,const unsigned char* data, int bytesRead)
{
    int idx = 0;
    packet[idx++] = 2;
    packet[idx++] = bytesRead / 256;
    packet[idx++] = bytesRead % 256;
    memcpy(&packet[idx],data,bytesRead);
    idx += bytesRead;
    return idx;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // ----------------------------------------------------
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

    if (strcmp(role, "tx") == 0)
    {
        FILE *target = fopen(filename, "r");
        if (target == NULL)
        {
            printf("Could not open %s\n", filename);
            return;
        }

        fseek(target, 0, SEEK_END);
        long fileSize = ftell(target);
        fseek(target, 0, SEEK_SET);

        if (llOpenTx(llParameters) != 0)
        {
            return;
        }

        unsigned char buffer[256];
        int packetSize = buildControlPacket(buffer, 1, fileSize, filename);
        llSend(buffer, packetSize);


        unsigned char dataBuffer[1021];
        unsigned char dataPacketBuffer[1024];
        int bytesRead;

        while ((bytesRead = fread(dataBuffer,1, sizeof(dataBuffer), target)) > 0){
            int packetSize = buildDataPacket(dataPacketBuffer,dataBuffer, bytesRead);
            llSend(dataPacketBuffer,packetSize);
        }

        packetSize = buildControlPacket(buffer, 3, fileSize,filename);
        llSend(buffer, packetSize);

    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) != 0)
        {
            return;
        }
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
