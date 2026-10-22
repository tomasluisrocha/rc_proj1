// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

#define DATA_PACKET_HEADER 3   // C + L2 + L1
#define MAX_DATA_FIELD (MAX_PAYLOAD_SIZE - DATA_PACKET_HEADER)   // 997

#define CTRL_START 1
#define CTRL_DATA  2
#define CTRL_END   3
#define TLV_SIZE   0
#define TLV_NAME   1

int buildControlPacket(unsigned char *packet, unsigned char controlType, long fileSize, const char *filename)
{
    int idx = 0;

    packet[idx++] = controlType;
    packet[idx++] = TLV_SIZE;

    unsigned char sizeBytes = sizeof(fileSize);
    packet[idx++] = sizeBytes;
    memcpy(&packet[idx], &fileSize, sizeBytes);
    idx += sizeBytes;

    packet[idx++] = TLV_NAME;
    int nameLen = strlen(filename);
    packet[idx++] = (unsigned char)nameLen;
    memcpy(&packet[idx], filename, nameLen);
    idx += nameLen;

    return idx;
}

int buildDataPacket(unsigned char *packet, const unsigned char *data, int bytesRead)
{
    int idx = 0;
    packet[idx++] = CTRL_DATA;
    packet[idx++] = bytesRead / 256;
    packet[idx++] = bytesRead % 256;
    memcpy(&packet[idx], data, bytesRead);
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
        FILE *target = fopen(filename, "rb");
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
            printf("Could not open connection as Transmitter\n");
            fclose(target);
            return;
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int packetSize = buildControlPacket(buffer, CTRL_START, fileSize, filename);
        if (llSend(buffer, packetSize) == -1)
        {
            printf("Could not send control packet\nClosing connection\n");
            fclose(target);
            llCloseTx();
            return;
        }

        unsigned char dataBuffer[MAX_DATA_FIELD];
        unsigned char dataPacketBuffer[MAX_PAYLOAD_SIZE];
        int bytesRead;

        while ((bytesRead = fread(dataBuffer, 1, sizeof(dataBuffer), target)) > 0)
        {
            int packetSize = buildDataPacket(dataPacketBuffer, dataBuffer, bytesRead);
            if (llSend(dataPacketBuffer, packetSize) == -1)
            {
                printf("Could not send data packet\nClosing connection\n");
                fclose(target);
                llCloseTx();
                return;
            }
        }

        packetSize = buildControlPacket(buffer, CTRL_END, fileSize, filename);
        if (llSend(buffer, packetSize) == -1)
        {
            printf("Could not send control packet\nClosing connection\n");
            fclose(target);
            llCloseTx();
            return;
        }

        printf("File %s sucessfully transmitted\n",filename);
        fclose(target);
        llCloseTx();
        return;
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
