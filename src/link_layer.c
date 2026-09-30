// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// Supervision Frame Stuff

#define FLAG_VALUE 0x7E
#define A_TX_CMD 0x03
#define A_RX_CMD 0x01
#define CTRL_SET 0x03
#define CTRL_UA 0x07

const unsigned char SET_FRAME[5] = {FLAG_VALUE, A_TX_CMD, CTRL_SET, A_TX_CMD ^ CTRL_SET, FLAG_VALUE};
const unsigned char UA_FRAME[5] = {FLAG_VALUE, A_TX_CMD, CTRL_UA, A_TX_CMD ^ CTRL_UA, FLAG_VALUE};

// Alarm stuff

int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d received\n", alarmCount);
}

// Receiver State Machine

typedef enum
{
    STATE_START,
    STATE_FLAG_RCV,
    STATE_A_RCV,
    STATE_C_RCV,
    STATE_BCC_OK,
    STATE_STOP
} State;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    /*     // ----------------------------------------------------
        // This example code shows how to open the serial port and send a string.
        // TODO: Adapt and extend this code according to the specifications of the project.
        // ----------------------------------------------------

    */
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    int receivedBytes = 0;
    int sentBytes = 0;
    alarmCount = 0;
    alarmEnabled = FALSE;

    int connectionEstabilished = FALSE;

    while (alarmCount < llParameters.nRetransmissions && !connectionEstabilished)
    {
        if (alarmEnabled == FALSE)
        {
            int setbytes = writeBytesSerialPort(SET_FRAME, 5);
            if (setbytes == 5)
            {
                sentBytes += setbytes;
                printf("sent: ");
                for (int i = 0; i < 5; i++)
                {
                    printf(" 0x%02X", SET_FRAME[i]);
                }
                printf("\n");
            }
            else
            {
                printf("Could not send SET frame\n");
                closeSerialPort();
                return -1;
            }

            alarm(llParameters.timeout);
            alarmEnabled = TRUE;
        }

        int frameIndex = 0;
        unsigned char buf[5] = {0};
        volatile int stopWaiting = FALSE;

        printf("received:");
        while (alarmEnabled == TRUE && !stopWaiting)
        {
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            if (bytes > 0)
            {
                buf[frameIndex] = byte;
                frameIndex++;
                receivedBytes++;
                printf(" 0x%02X", byte);
            }
            else if (bytes < 0)
            {
                break;
            }
            if (frameIndex == 5)
            {
                stopWaiting = TRUE;
            }
        }
        printf("\n");

        if (stopWaiting &&
            buf[0] == FLAG_VALUE &&
            buf[1] == A_TX_CMD &&
            buf[2] == CTRL_UA &&
            buf[3] == (buf[1] ^ buf[2]) &&
            buf[4] == FLAG_VALUE)
        {
            connectionEstabilished = TRUE;
        }
    }

    alarm(0);

    int result = 0;
    if (connectionEstabilished)
    {
        printf("Connection Estabilished");
    }
    else
    {
        printf("Connection failed: maximum retransmissions reached without valid UA");
        result = -1;
    }

    printf("Total bytes sent: %d\n", sentBytes);
    printf("Total bytes received: %d\n", receivedBytes);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return result;
}

int llOpenRx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and receive a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    int receivedBytes = 0;
    int sentBytes = 0;
    State state = STATE_START;

    unsigned char address_flag = 0;
    unsigned char control_flag = 0;
    // Receive SET_FRAME

    printf("received:");
    while (state != STATE_STOP)
    {
        unsigned char byte;
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0)
        {
            receivedBytes++;
            printf(" 0x%02X", byte);
            switch (state)
            {
            case STATE_START:
                if (byte == FLAG_VALUE)
                {
                    state = STATE_FLAG_RCV;
                }
                break;
            case STATE_FLAG_RCV:
                if (byte == FLAG_VALUE)
                {
                    state = STATE_FLAG_RCV;
                }
                else if (byte == A_TX_CMD)
                {
                    address_flag = byte;
                    state = STATE_A_RCV;
                }
                else
                {
                    state = STATE_START;
                }
                break;
            case STATE_A_RCV:
                if (byte == FLAG_VALUE)
                {
                    state = STATE_FLAG_RCV;
                }
                else if (byte == CTRL_SET)
                {
                    control_flag = byte;
                    state = STATE_C_RCV;
                }
                else
                {
                    state = STATE_START;
                }
                break;
            case STATE_C_RCV:
                if (byte == (address_flag ^ control_flag))
                {
                    state = STATE_BCC_OK;
                }
                else if (byte == FLAG_VALUE)
                {
                    state = STATE_FLAG_RCV;
                }
                else
                {
                    state = STATE_START;
                }
                break;
            case STATE_BCC_OK:
                if (byte == FLAG_VALUE)
                {
                    state = STATE_STOP;
                }
                else
                {
                    state = STATE_START;
                }
                break;
            default:
                state = STATE_START;
                break;
            }
        }
        else if (bytes < 0)
        {
            perror("readByteSerialPort");
            break;
        }
    }
    printf("\n");

    int result = 0;
    if (state == STATE_STOP)
    {
        int uaBytes = writeBytesSerialPort(UA_FRAME, 5);
        if (uaBytes == 5)
        {
            sentBytes += uaBytes;
            printf("sent:");
            for (int i = 0; i < 5; i++)
            {
                printf(" 0x%02X", UA_FRAME[i]);
            }
            printf("\n");
            printf("Connection Established\n");
        }
        else
        {
            printf("Connection failed: could not send UA frame\n");
            result = -1;
        }
    }
    else
    {
        printf("Connection failed: invalid frame sequence\n");
        result = -1;
    }

    printf("Total bytes sent: %d\n", sentBytes);
    printf("Total bytes received: %d\n", receivedBytes);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return result;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}
