// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <signal.h>

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
    STATE_STOP
} State;

typedef struct Stats
{
    LinkLayer params;
    int ns;
    int bytesSent, bytesReceived;
    int framesSent, framesReceived, rejReceived;
} Stats;

static Stats stats;

// Write frame to serial and return number of written chars
int writeFrame(const unsigned char *frame, int n)
{
    int w = writeBytesSerialPort(frame, n);
    return w;
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

    printf("Serial port %s opened\n", llParameters.serialPort);

    alarmCount = 0;
    alarmEnabled = FALSE;

    int connectionEstablished = FALSE;

    while (alarmCount < llParameters.nRetransmissions && !connectionEstablished)
    {
        if (alarmEnabled == FALSE)
        {
            if (writeFrame(SET_FRAME, 5) != 5)
            {
                printf("Could not send SET frame\n");
                alarm(0);
                closeSerialPort();
                return -1;
            }
            stats.framesSent++;
            printf("Sent SET frame (attempt %d of %d)\n", alarmCount + 1, llParameters.nRetransmissions);
            alarm(llParameters.timeout);
            alarmEnabled = TRUE;
        }

        int c = readSupervision(A_TX_CMD, CTRL_UA, TRUE);
        if (c == CTRL_UA)
        {
            printf("Received UA frame\n");
            connectionEstablished = TRUE;
        }
    }

    alarm(0);

    if (!connectionEstablished)
    {
        printf("Connection failed: no valid UA received\n");
        closeSerialPort();
        return -1;
    }

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    stats = (Stats){.params = llParameters};

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened\n", llParameters.serialPort);

    if (readSupervision(A_TX_CMD, CTRL_SET, FALSE) != CTRL_SET)
    {
        printf("Connection failed: could not read SET frame\n");
        closeSerialPort();
        return -1;
    }
    printf("Received SET frame\n");

    if (writeFrame(UA_RX_FRAME, 5) != 5)
    {
        printf("Connection failed: could not send UA frame\n");
        closeSerialPort();
        return -1;
    }
    stats.framesSent++;

    printf("Connection established\n");
    return 0;
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
                printf("Could not send DISC frame\n");
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
            printf("Sent DISC frame to Rx");
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
        printf("Termination handshake complete.\n");
    }
    else
    {
        printf("Termination handshake timed-out.\n");
    }
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }
    printf("Transmitter serial port closed successfully.\n");
    return discReceived ? 0 : -1;
}

int llCloseRx()
{

    // Receive DISC frame
    if (readSupervision(A_TX_CMD, CTRL_DISC, FALSE) != CTRL_DISC)
    {
        printf("Error reading DISC frame from Tx\n");
        closeSerialPort();
        return -1;
    }
    printf("Received DISC frame from Tx\n");

    // Send DISC frame
    if (writeFrame(DISC_RX_FRAME, 5) != 5)
    {
        printf("Could not send DISC frame from Rx\n");
        closeSerialPort();
        return -1;
    }
    stats.framesSent++;
    printf("Sent DISC frame back to Tx\n");

    // Await UA
    if (readSupervision(A_RX_CMD, CTRL_UA, FALSE) != CTRL_UA)
    {
        printf("Error reading final UA frame from Tx\n");
        closeSerialPort();
        return -1;
    }
    printf("Received UA frame from Tx. Termination handshake complete.\n");
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }
    printf("Receiver serial port closed successfully.\n");
    return 0;
}
