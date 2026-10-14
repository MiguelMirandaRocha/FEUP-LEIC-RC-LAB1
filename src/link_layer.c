// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

#define SUPERVISION_FRAME_SIZE 5

// Flag
#define FLAG 0x7E

// A Address
#define A_TX 0x03 // Commands from tx and replies from rx
#define A_RX 0x01 // commands from rx and replies from tx 

// C Control
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B

#define C_RR0 0xAA
#define C_RR1 0xAB

#define C_REJ0 0x54
#define C_REJ1 0x55

#define C_ns0 0x00
#define C_ns1 0x80

// Stuffing
#define ESC 0x7D
#define ESC_XOR 0x20


//states
typedef enum {
    START, 
    FLAG_RCV, 
    A_RCV, 
    C_RCV, 
    BCC1_OK, 
    ESC_RCV,
    DATA,
    STOP
} State; 

int alarmEnabled = FALSE; 
int alarmCount = 0; 

static unsigned char txNs;
static unsigned char rxExpectedNs;

typedef struct {
    // Tx
    int iFramesSent;
    int rejReceived;
    int timeouts;

    // Rx side
    int iFramesReceived;
    int duplicates;
    int bcc2Errors;
    int rrSent;
    int rejSent;
} Statistics;

typedef struct {
    State state;
    unsigned char expectedA;
    unsigned char c;
    unsigned char data[MAX_PAYLOAD_SIZE + 1];
    int dataLen;
} FrameParser;

static Statistics stats = {0};


static int isKnownC (unsigned char byte){
    return (byte == C_UA || byte == C_SET || byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == C_ns0 || byte == C_ns1 || byte == C_DISC);
}

static int isIFrame(unsigned char c) {
    return (c == C_ns0 || c == C_ns1);
}

// Appends a (destuffed) byte to the frame data and moves to DATA.
// data holds the payload plus BCC2, so it fits at most MAX_PAYLOAD_SIZE + 1 bytes.
// If the frame is too big it can't be valid: drop it and wait for the next one.
static int storeByte(FrameParser *p, unsigned char byte){
    if (p->dataLen < 0 || p->dataLen >= (int)sizeof(p->data)){
        p->dataLen = 0;
        p->state = START;
        return -1;
    }
    p->data[p->dataLen++] = byte;
    p->state = DATA;
    return 0;
}

static void nextState(FrameParser *p, unsigned char byte){
    switch (p->state){    
        case START:
            if (byte == FLAG) p->state = FLAG_RCV;
            break; 

        case FLAG_RCV:
            if (byte == p->expectedA) p->state = A_RCV; 
            else if (byte == FLAG) p->state = FLAG_RCV;
            else p->state= START; 
            break; 

        case A_RCV: 
            if (isKnownC(byte)){p->c = byte; p->state = C_RCV;}
            else if (byte == FLAG) p->state = FLAG_RCV; 
            else p->state = START; 
            break; 

        case C_RCV: 
            if (byte == (p->expectedA ^ p->c)) p->state = BCC1_OK; 
            else if (byte == FLAG) p->state = FLAG_RCV; 
            else p->state = START; 
            break; 

        case BCC1_OK: 
            if (!isIFrame(p->c)){
                if (byte == FLAG) p->state = STOP;
                else p->state = START;
            } 
            else {
                if (byte == FLAG) p->state = FLAG_RCV;
                else if (byte == ESC) p->state = ESC_RCV;
                else storeByte(p, byte);
            }
            break;

        case DATA:
            if (byte == ESC) p->state = ESC_RCV;
            else if (byte == FLAG) p->state = STOP;
            else storeByte(p, byte);
            break;

        case ESC_RCV:
            if (byte == FLAG) {
                p->dataLen = 0;
                p->state = FLAG_RCV;
            }
            // Only 0x7E and 0x7D are ever escaped (sent as 0x5E / 0x5D)
            else if (byte == (FLAG ^ ESC_XOR) || byte == (ESC ^ ESC_XOR)) {
                storeByte(p, byte ^ ESC_XOR);
            }
            else {
                p->dataLen = 0;
                p->state = START;
            }
            break;
        case STOP: 
            break; 
    }
}


static void resetParser(FrameParser *p, unsigned char expectedA){
    p->state = START;
    p->dataLen = 0; 
    p->expectedA = expectedA;
}


static void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++; 
}



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened\n", llParameters.serialPort);



    // Create SET array 
    unsigned char buf[SUPERVISION_FRAME_SIZE] = {0};
    buf[0] = FLAG; 
    buf[1] = A_TX; 
    buf[2] = C_SET; 
    buf[3] = A_TX ^ C_SET; 
    buf[4] = FLAG; 


    // ALARM CONFIGURATION
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1){
        perror("sigaction");
        exit(1);
    } 
    

    FrameParser p; 
    resetParser(&p, A_TX);
    alarmEnabled = FALSE; 
    alarmCount = 0; 
    // 1 original send + nRetransmissions retries
    while (alarmCount <= llParameters.nRetransmissions && p.state != STOP){
        if (alarmEnabled == FALSE){
            writeBytesSerialPort(buf, SUPERVISION_FRAME_SIZE);
            alarm(llParameters.timeout);
            alarmEnabled = TRUE; 
            resetParser(&p, A_TX);
        }
        unsigned char byte; 
        if (readByteSerialPort(&byte) > 0){
            nextState(&p, byte);
            // Valid frame but not a UA: ignore it and keep listening
            if (p.state == STOP && p.c != C_UA) resetParser(&p, A_TX);
        }
    }
    alarm(0);

    // No UA after all attempts, connection not established
    if (p.state != STOP){
        printf("No UA after %d attempts\n", alarmCount);
        if (closeSerialPort() < 0)
        {
            perror("closeSerialPort");
            return -1;
        }
        return -1; 
    }

    //UA received, connectione stablished. exit w success
    printf("UA Received\n");


    /*
    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);
    */

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);
    FrameParser p; 
    resetParser(&p, A_TX);
    while (p.state != STOP){
        unsigned char byte;
        if (readByteSerialPort(&byte) > 0){
            nextState(&p, byte);
            // Valid frame but not a SET: ignore it and keep listening
            if (p.state == STOP && p.c != C_SET) resetParser(&p, A_TX);
        }
    }
    printf("SET received\n");


    // Create UA array 
    unsigned char buf[SUPERVISION_FRAME_SIZE] = {0};
    buf[0] = FLAG; 
    buf[1] = A_TX; 
    buf[2] = C_UA; 
    buf[3] = A_TX ^ C_UA; 
    buf[4] = FLAG; 

    int bytes = writeBytesSerialPort(buf, SUPERVISION_FRAME_SIZE);
    printf("%d bytes written to serial port\n", bytes);

    /*
    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    */
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
    // TODO: Implement this function

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}
