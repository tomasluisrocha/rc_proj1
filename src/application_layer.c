// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>
#include <limits.h>

#define DATA_PACKET_HEADER 3                                   // C + L2 + L1
#define MAX_DATA_FIELD (MAX_PAYLOAD_SIZE - DATA_PACKET_HEADER) // 997

#define CTRL_START 1
#define CTRL_DATA 2
#define CTRL_END 3
#define TLV_SIZE 0
#define TLV_NAME 1

static void printProgress(const char *label, long current, long total)
{
    long percent = total > 0 ? (current * 100) / total : 100;
    if (percent > 100)
        percent = 100;
    printf("\r%s: %ld/%ld bytes (%ld%%)", label, current, total, percent);
    fflush(stdout);
}

int buildControlPacket(unsigned char *packet, unsigned char controlType, long fileSize, const char *filename)
{
    if (packet == NULL || filename == NULL ||
        (controlType != CTRL_START && controlType != CTRL_END) ||
        fileSize < 0)
        return -1;

    size_t nameLen = strlen(filename);
    if (nameLen > UCHAR_MAX ||
        1 + 2 + sizeof(fileSize) + 2 + nameLen > MAX_PAYLOAD_SIZE)
        return -1;

    int idx = 0;

    packet[idx++] = controlType;
    packet[idx++] = TLV_SIZE;

    unsigned char sizeBytes = sizeof(fileSize);
    packet[idx++] = sizeBytes;
    memcpy(&packet[idx], &fileSize, sizeBytes);
    idx += sizeBytes;

    packet[idx++] = TLV_NAME;
    packet[idx++] = (unsigned char)nameLen;
    memcpy(&packet[idx], filename, nameLen);
    idx += nameLen;

    return idx;
}

int parseControlPacket(const unsigned char *packet, int size, long *fileSize, char *name, int nameMax)
{
    if (packet == NULL || fileSize == NULL || name == NULL || size < 3 ||
        size > MAX_PAYLOAD_SIZE || nameMax <= 0 ||
        (packet[0] != CTRL_START && packet[0] != CTRL_END))
        return -1;

    int gotSize = FALSE;
    int gotName = FALSE;
    *fileSize = -1;
    name[0] = '\0';
    int idx = 1;
    while (idx < size)
    {
        if (size - idx < 2)
            return -1;

        unsigned char type = packet[idx++];
        unsigned char length = packet[idx++];
        if (length > size - idx)
            return -1;

        if (type == 0)
        {
            if (gotSize || length != sizeof(*fileSize))
                return -1;
            memcpy(fileSize, &packet[idx], length);
            if (*fileSize < 0)
                return -1;
            gotSize = TRUE;
        }
        else if (type == 1)
        {
            if (gotName || length >= nameMax)
                return -1;
            memcpy(name, &packet[idx], length);
            name[length] = '\0';
            gotName = TRUE;
        }
        idx += length;
    }

    return gotSize && gotName ? 0 : -1;
}

int buildDataPacket(unsigned char *packet, const unsigned char *data, int bytesRead)
{
    if (packet == NULL || data == NULL || bytesRead < 0 ||
        bytesRead > MAX_DATA_FIELD)
        return -1;

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
            printf("Error: could not open %s\n", filename);
            return;
        }

        if (fseek(target, 0, SEEK_END) != 0)
        {
            printf("Error: could not determine size of %s\n", filename);
            fclose(target);
            return;
        }
        long fileSize = ftell(target);
        if (fileSize < 0 || fseek(target, 0, SEEK_SET) != 0)
        {
            printf("Error: could not determine size of %s\n", filename);
            fclose(target);
            return;
        }

        if (llOpenTx(llParameters) != 0)
        {
            printf("Error: could not open transmitter connection\n");
            fclose(target);
            return;
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int packetSize = buildControlPacket(buffer, CTRL_START, fileSize, filename);
        if (packetSize < 0 || llSend(buffer, packetSize) == -1)
        {
            printf("Error: could not send START packet\n");
            fclose(target);
            llCloseTx();
            return;
        }

        unsigned char dataBuffer[MAX_DATA_FIELD];
        unsigned char dataPacketBuffer[MAX_PAYLOAD_SIZE];
        int bytesRead;
        long bytesSent = 0;

        while ((bytesRead = fread(dataBuffer, 1, sizeof(dataBuffer), target)) > 0)
        {
            int packetSize = buildDataPacket(dataPacketBuffer, dataBuffer, bytesRead);
            if (packetSize < 0 || llSend(dataPacketBuffer, packetSize) == -1)
            {
                printf("Error: could not send data packet\n");
                fclose(target);
                llCloseTx();
                return;
            }
            bytesSent += bytesRead;
            printProgress("Sent", bytesSent, fileSize);
        }

        if (ferror(target))
        {
            printf("\nError: could not read %s\n", filename);
            fclose(target);
            llCloseTx();
            return;
        }

        packetSize = buildControlPacket(buffer, CTRL_END, fileSize, filename);
        if (packetSize < 0 || llSend(buffer, packetSize) == -1)
        {
            printf("\nError: could not send END packet\n");
            fclose(target);
            if (llCloseTx() != 0)
                printf("Error: transmission termination failed\n");
            return;
        }

        printf("\nSent %ld bytes from %s\n", fileSize, filename);
        fclose(target);
        llCloseTx();
        return;
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) != 0)
        {
            printf("Error: could not open receiver connection\n");
            return;
        }

        unsigned char packet[MAX_PAYLOAD_SIZE];
        char startName[256], endName[256];
        long startSize = -1, endSize = -1;
        long bytesWritten = 0;
        int packetSize;

        // Wait for the START packet
        do
        {
            packetSize = llReceive(packet);
            if (packetSize <= 0)
            {
                printf("Error: could not receive START packet\n");
                llCloseRx();
                return;
            }
        } while (packet[0] != CTRL_START);

        if (parseControlPacket(packet, packetSize, &startSize, startName, sizeof(startName)) != 0)
        {
            printf("Error: invalid START packet\n");
            llCloseRx();
            return;
        }
        if (startSize < 0)
        {
            printf("Error: invalid START file size\n");
            llCloseRx();
            return;
        }
        printf("Receiving %s (%ld bytes)\n", startName, startSize);

        FILE *output = fopen(filename, "wb");
        if (output == NULL)
        {
            printf("Error: could not create %s\n", filename);
            llCloseRx();
            return;
        }

        // Receive DATA packets until END
        while (TRUE)
        {
            packetSize = llReceive(packet);
            if (packetSize <= 0)
            {
                printf("\nError: could not receive packet\n");
                fclose(output);
                llCloseRx();
                return;
            }

            if (packet[0] == CTRL_END)
                break;

            if (packet[0] != CTRL_DATA)
            {
                printf("\nError: unexpected packet type %d\n", packet[0]);
                fclose(output);
                llCloseRx();
                return;
            }

            // Check packet size
            if (packetSize < DATA_PACKET_HEADER)
            {
                printf("\nError: data packet too short\n");
                fclose(output);
                llCloseRx();
                return;
            }

            int dataSize = packet[1] * 256 + packet[2]; // K = 256 * L2 + L1
            if (dataSize != packetSize - DATA_PACKET_HEADER)
            {
                printf("\nError: data packet size mismatch\n");
                fclose(output);
                llCloseRx();
                return;
            }

            if (fwrite(&packet[DATA_PACKET_HEADER], 1, dataSize, output) != (size_t)dataSize)
            {
                printf("\nError: could not write to %s\n", filename);
                fclose(output);
                llCloseRx();
                return;
            }
            bytesWritten += dataSize;
            printProgress("Received", bytesWritten, startSize);
        }

        fclose(output);

        // Check if END matches START packet
        if (parseControlPacket(packet, packetSize, &endSize, endName, sizeof(endName)) != 0 ||
            endSize != startSize || strcmp(startName, endName) != 0)
        {
            printf("\nError: END packet does not match START packet\n");
            if (llCloseRx() != 0)
                printf("Error: reception termination failed\n");
            return;
        }

        if (bytesWritten != startSize)
        {
            printf("\nError: file size mismatch (expected %ld, received %ld)\n",
                   startSize, bytesWritten);
            llCloseRx();
            return;
        }

        printf("\nReceived %ld bytes into %s\n", bytesWritten, filename);
        llCloseRx();
        return;
    }
    else
    {
        printf("Error: invalid role: %s (must be 'tx' or 'rx')\n", role);
        return;
    }
}
