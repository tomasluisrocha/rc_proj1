// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

#define ANY_CTRL -1

// Supervision Frame Stuff

#define FLAG_VALUE 0x7E
#define A_TX_CMD 0x03
#define A_RX_CMD 0x01
#define CTRL_SET 0x03
#define CTRL_UA 0x07
#define CTRL_DISC 0x0B
#define CTRL_I0 0x00
#define CTRL_I1 0x80
#define CTRL_RR 0xAA
#define CTRL_REJ 0x54
#define ESC_VALUE 0x7D
#define ESC_XOR 0x20

const unsigned char SET_FRAME[5] = {FLAG_VALUE, A_TX_CMD, CTRL_SET, A_TX_CMD ^ CTRL_SET, FLAG_VALUE};
const unsigned char UA_RX_FRAME[5] = {FLAG_VALUE, A_TX_CMD, CTRL_UA, A_TX_CMD ^ CTRL_UA, FLAG_VALUE};
const unsigned char UA_TX_FRAME[5] = {FLAG_VALUE, A_RX_CMD, CTRL_UA, A_RX_CMD ^ CTRL_UA, FLAG_VALUE};
const unsigned char DISC_TX_FRAME[5] = {FLAG_VALUE, A_TX_CMD, CTRL_DISC, A_TX_CMD ^ CTRL_DISC, FLAG_VALUE};
const unsigned char DISC_RX_FRAME[5] = {FLAG_VALUE, A_RX_CMD, CTRL_DISC, A_RX_CMD ^ CTRL_DISC, FLAG_VALUE};

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

typedef enum State
{
    STATE_START,
    STATE_FLAG_RCV,
    STATE_A_RCV,
    STATE_C_RCV,
    STATE_DATA,
    STATE_STOP
} State;

typedef struct Stats
{
    LinkLayer params;
    int ns;
    int bytesSent, bytesReceived;
    int framesSent, framesReceived, rejReceived, rejSent;
    int retransmissions, timeouts, duplicates;
} Stats;

static Stats stats;

static void printStats(void)
{
    printf("Statistics:\n"
           "  - Bytes sent: %d\n"
           "  - Bytes received: %d\n"
           "  - Frames sent: %d\n"
           "  - Frames received: %d\n"
           "  - Retransmissions: %d\n"
           "  - Timeouts: %d\n"
           "  - REJ sent: %d\n"
           "  - REJ received: %d\n"
           "  - duplicates: %d\n",
           stats.bytesSent,
           stats.bytesReceived, stats.framesSent,
           stats.framesReceived, stats.retransmissions, stats.timeouts,
           stats.rejSent, stats.rejReceived, stats.duplicates);
}

// Write frame to serial and return number of written chars
int writeFrame(const unsigned char *frame, int n)
{
    int w = writeBytesSerialPort(frame, n);
    if (w > 0)
        stats.bytesSent += w;
    return w;
}

static int stuffByte(unsigned char *out, unsigned char byte)
{
    if (byte == FLAG_VALUE || byte == ESC_VALUE)
    {
        out[0] = ESC_VALUE;
        out[1] = byte ^ ESC_XOR;
        return 2;
    }
    out[0] = byte;
    return 1;
}

static int sendReply(unsigned char control)
{
    unsigned char frame[5] = {FLAG_VALUE, A_TX_CMD, control,
                              A_TX_CMD ^ control, FLAG_VALUE};
    if (writeFrame(frame, sizeof(frame)) != (int)sizeof(frame))
        return -1;
    stats.framesSent++;
    return 0;
}

// Read supervision frame and compare it to addr/control bytes
// Returns the control byte
int readSupervision(unsigned char addr, int control, int timed)
{
    State state = STATE_START;
    unsigned char received_ctrl = 0, bcc = 0, byte;
    while (state != STATE_STOP && (!timed || alarmEnabled == TRUE))
    {
        int n = readByteSerialPort(&byte);
        if (n < 0)
            return -1;
        if (n == 0)
            continue;
        stats.bytesReceived++;

        switch (state)
        {
        case STATE_START: // State 0
            if (byte == FLAG_VALUE)
                state = STATE_FLAG_RCV;
            break;
        case STATE_FLAG_RCV: // State 1
            if (byte == addr)
                state = STATE_A_RCV;
            else if (byte != FLAG_VALUE)
                state = STATE_START;
            break;
        case STATE_A_RCV: // State 2
            if (control == ANY_CTRL || byte == control)
            {
                received_ctrl = byte;
                bcc = 0;
                state = STATE_C_RCV;
            }
            else
                state = STATE_START;
            break;
        case STATE_C_RCV: // State 3
            if (byte != FLAG_VALUE)
                bcc = byte;
            else if (bcc == (addr ^ received_ctrl)) // State 4
            {
                state = STATE_STOP;
                stats.framesReceived++;
            }
            else
                state = STATE_START;
            break;
        default:
            state = STATE_START;
            break;
        }
    }
    return state == STATE_STOP ? received_ctrl : -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    stats = (Stats){.params = llParameters};

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

    alarmCount = 0;
    alarmEnabled = FALSE;

    int connectionEstablished = FALSE;

    while (alarmCount < llParameters.nRetransmissions && !connectionEstablished)
    {
        if (alarmEnabled == FALSE)
        {
            if (writeFrame(SET_FRAME, 5) != 5)
            {
                printf("Error: could not send SET frame\n");
                alarm(0);
                closeSerialPort();
                return -1;
            }
            stats.framesSent++;
            alarm(llParameters.timeout);
            alarmEnabled = TRUE;
        }

        int c = readSupervision(A_TX_CMD, CTRL_UA, TRUE);
        if (c == CTRL_UA)
        {
            connectionEstablished = TRUE;
        }
    }

    alarm(0);

    if (!connectionEstablished)
    {
        printf("Error: connection failed: no valid UA received\n");
        closeSerialPort();
        return -1;
    }

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    stats = (Stats){.params = llParameters};

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
    if (readSupervision(A_TX_CMD, CTRL_SET, FALSE) != CTRL_SET)
    {
        printf("Error: connection failed: could not read SET frame\n");
        closeSerialPort();
        return -1;
    }
    if (writeFrame(UA_RX_FRAME, 5) != 5)
    {
        printf("Error: connection failed: could not send UA frame\n");
        closeSerialPort();
        return -1;
    }
    stats.framesSent++;

    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    if (buf == NULL || bufSize <= 0 || bufSize > MAX_PAYLOAD_SIZE ||
        stats.params.nRetransmissions <= 0 || stats.params.timeout <= 0)
        return -1;

    unsigned char frame[2 * (MAX_PAYLOAD_SIZE + 1) + 6];
    int frameSize = 0;
    unsigned char control = stats.ns == 0 ? CTRL_I0 : CTRL_I1;
    unsigned char bcc2 = 0;

    frame[frameSize++] = FLAG_VALUE;
    frame[frameSize++] = A_TX_CMD;
    frame[frameSize++] = control;
    frame[frameSize++] = A_TX_CMD ^ control;

    for (int i = 0; i < bufSize; i++)
    {
        bcc2 ^= buf[i];
        frameSize += stuffByte(&frame[frameSize], buf[i]);
    }
    frameSize += stuffByte(&frame[frameSize], bcc2);
    frame[frameSize++] = FLAG_VALUE;

    int attempts = 0;
    int accepted = FALSE;
    alarmEnabled = FALSE;
    alarmCount = 0;

    while (attempts < stats.params.nRetransmissions && !accepted)
    {
        if (!alarmEnabled)
        {
            if (writeFrame(frame, frameSize) != frameSize)
            {
                alarm(0);
                return -1;
            }
            stats.framesSent++;
            if (attempts > 0)
                stats.retransmissions++;
            attempts++;
            alarm(stats.params.timeout);
            alarmEnabled = TRUE;
        }

        int reply = readSupervision(A_TX_CMD, ANY_CTRL, TRUE);
        if (reply == (CTRL_RR | (control == CTRL_I0 ? 1 : 0)))
        {
            accepted = TRUE;
        }
        else if (reply == (CTRL_REJ | (control == CTRL_I0 ? 0 : 1)))
        {
            stats.rejReceived++;
            alarm(0);
            alarmEnabled = FALSE;
        }
        else if (reply < 0 && alarmEnabled)
        {
            alarmEnabled = FALSE;
            stats.timeouts++;
        }
    }

    alarm(0);
    alarmEnabled = FALSE;
    if (!accepted)
        return -1;

    stats.ns ^= 1;
    return bufSize;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    if (packet == NULL)
        return -1;

    unsigned char data[MAX_PAYLOAD_SIZE + 1];
    State state = STATE_START;
    unsigned char control = 0;
    int length = 0;
    int escaped = FALSE;

    while (TRUE)
    {
        unsigned char byte;
        int n = readByteSerialPort(&byte);
        if (n < 0)
            return -1;
        if (n == 0)
            continue;
        stats.bytesReceived++;

        switch (state)
        {
        case STATE_START:
            if (byte == FLAG_VALUE)
                state = STATE_FLAG_RCV;
            break;
        case STATE_FLAG_RCV:
            if (byte == A_TX_CMD)
                state = STATE_A_RCV;
            else if (byte != FLAG_VALUE)
                state = STATE_START;
            break;
        case STATE_A_RCV:
            if (byte == CTRL_I0 || byte == CTRL_I1 || byte == CTRL_SET)
            {
                control = byte;
                state = STATE_C_RCV;
            }
            else
                state = STATE_START;
            break;
        case STATE_C_RCV:
            if (byte == (A_TX_CMD ^ control))
            {
                length = 0;
                escaped = FALSE;
                state = STATE_DATA;
            }
            else
                state = STATE_START;
            break;
        case STATE_DATA:
            if (byte == FLAG_VALUE)
            {
                if (escaped)
                {
                    state = STATE_START;
                    break;
                }

                if (control == CTRL_SET)
                {
                    if (writeFrame(UA_RX_FRAME, sizeof(UA_RX_FRAME)) !=
                        (int)sizeof(UA_RX_FRAME))
                        return -1;
                    stats.framesSent++;
                    state = STATE_START;
                    break;
                }

                int frameNs = control == CTRL_I0 ? 0 : 1;
                if (length < 1)
                {
                    state = STATE_START;
                    break;
                }

                unsigned char bcc2 = 0;
                for (int i = 0; i < length - 1; i++)
                    bcc2 ^= data[i];

                if (bcc2 != data[length - 1])
                {
                    if (frameNs == stats.ns)
                    {
                        stats.rejSent++;
                        if (sendReply(CTRL_REJ | stats.ns) < 0)
                            return -1;
                    }
                    else
                    {
                        stats.duplicates++;
                        if (sendReply(CTRL_RR | stats.ns) < 0)
                            return -1;
                    }
                    state = STATE_START;
                    break;
                }

                if (frameNs != stats.ns)
                {
                    stats.duplicates++;
                    if (sendReply(CTRL_RR | stats.ns) < 0)
                        return -1;
                    state = STATE_START;
                    break;
                }

                memcpy(packet, data, length - 1);
                stats.ns ^= 1;
                stats.framesReceived++;
                if (sendReply(CTRL_RR | stats.ns) < 0)
                    return -1;
                return length - 1;
            }

            if (byte == ESC_VALUE)
            {
                if (escaped)
                {
                    state = STATE_START;
                    break;
                }
                escaped = TRUE;
                break;
            }

            if (escaped)
            {
                if (byte != (FLAG_VALUE ^ ESC_XOR) &&
                    byte != (ESC_VALUE ^ ESC_XOR))
                {
                    state = STATE_START;
                    break;
                }
                byte ^= ESC_XOR;
                escaped = FALSE;
            }

            if (length >= (int)sizeof(data))
            {
                state = STATE_START;
                break;
            }
            data[length++] = byte;
            break;
        default:
            state = STATE_START;
            break;
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    alarmCount = 0;
    alarmEnabled = FALSE;
    int discReceived = FALSE;

    // Send DISC frame
    while (alarmCount < stats.params.nRetransmissions && !discReceived)
    {
        if (alarmEnabled == FALSE)
        {
            if (writeFrame(DISC_TX_FRAME, 5) != 5)
            {
                printf("Error: could not send DISC frame\n");
                alarm(0);
                closeSerialPort();
                return -1;
            }
            stats.framesSent++;
            alarm(stats.params.timeout);
            alarmEnabled = TRUE;
        }

        // Receive DISC frame
        int c = readSupervision(A_RX_CMD, CTRL_DISC, TRUE);
        if (c == CTRL_DISC)
        {
            discReceived = TRUE;
        }
    }
    alarm(0);

    // Write UA frame
    if (discReceived)
    {
        if (writeFrame(UA_TX_FRAME, 5) == 5)
        {
            stats.framesSent++;
        }
    }
    else
    {
        printf("Error: link termination timed out\n");
    }
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }
    printStats();
    return discReceived ? 0 : -1;
}

int llCloseRx()
{

    // Receive DISC frame
    if (readSupervision(A_TX_CMD, CTRL_DISC, FALSE) != CTRL_DISC)
    {
        printf("Error: could not read DISC frame from Tx\n");
        closeSerialPort();
        return -1;
    }
    alarmCount = 0;
    alarmEnabled = FALSE;
    int uaReceived = FALSE;
    while (alarmCount < stats.params.nRetransmissions && !uaReceived)
    {
        if (!alarmEnabled)
        {
            if (writeFrame(DISC_RX_FRAME, 5) != 5)
            {
                printf("Error: could not send DISC frame from Rx\n");
                alarm(0);
                closeSerialPort();
                return -1;
            }
            stats.framesSent++;
            alarm(stats.params.timeout);
            alarmEnabled = TRUE;
        }

        if (readSupervision(A_RX_CMD, CTRL_UA, TRUE) == CTRL_UA)
            uaReceived = TRUE;
        else if (!alarmEnabled)
            stats.timeouts++;
    }
    alarm(0);
    alarmEnabled = FALSE;

    if (!uaReceived)
    {
        printf("Error: could not read final UA frame from Tx\n");
        closeSerialPort();
        return -1;
    }
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }
    printStats();
    return 0;
}
