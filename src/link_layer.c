// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

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

 #define NO_REJ -1


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

static volatile sig_atomic_t timedOut = FALSE; 

static LinkLayer params; 
static unsigned char txNs;
static unsigned char rxExpectedNs;

typedef struct {
    // Tx
    int iFramesSent;
    int retransmissions;
    int rejReceived;
    int timeouts;

    // Rx side
    int iFramesReceived;
    int duplicates;
    int bcc2Errors;
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

static int writeAll(const unsigned char *buf, int n){
    int sent = 0;
    while (sent < n){
        int w = writeBytesSerialPort(buf + sent, n - sent);
        if (w < 0) return -1;
        sent += w;
    }
    return n;
}

// Sends a supervision frame: F A C BCC1 F
static int sendSupervision(unsigned char a, unsigned char c){
    unsigned char buf[SUPERVISION_FRAME_SIZE] = {FLAG, a, c, a ^ c, FLAG};
    return writeAll(buf, SUPERVISION_FRAME_SIZE);
}


static void resetParser(FrameParser *p, unsigned char expectedA){
    p->state = START;
    p->dataLen = 0; 
    p->expectedA = expectedA;
}


static void alarmHandler(int signal)
{
    timedOut = TRUE; 
}
static void setupAlarm(){
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1){
        perror("sigaction");
        exit(1);
    }
} 

static int sendAndWait(const unsigned char *frame, int len, unsigned char expectedA, unsigned char ackC, int rejC){
    FrameParser p;
    int attempts = 0;
    int resend = TRUE;
    timedOut = FALSE;

    while (TRUE) {
        if (timedOut){
            timedOut = FALSE;
            resend = TRUE;
            stats.timeouts++;
        }

        if (resend){
            if (attempts++ > params.nRetransmissions){
                alarm(0);
                return -1;
            }
            if (attempts > 1) stats.retransmissions++;

            if (writeAll(frame, len) < 0){
                alarm(0);
                return -1;
            }


            alarm(params.timeout);
            resetParser(&p, expectedA);
            resend = FALSE;
        }

        unsigned char byte;
        if (readByteSerialPort(&byte) <= 0) continue;
        nextState(&p, byte);
        if (p.state != STOP) continue;

        if (p.c == ackC) {
            alarm(0);
            return 0;
        }

        if (p.c == rejC){
            stats.rejReceived++;
            resend = TRUE;
        }

        else resetParser(&p, expectedA);
    }
}

static int readFrame (FrameParser *p, unsigned char expectedA, int timeout){
    resetParser(p, expectedA);
    timedOut = FALSE; 
    alarm (timeout); 

    while(p->state != STOP){
        if (timedOut) break; 
        unsigned char byte;
        int r = readByteSerialPort(&byte);
        if (r < 0 && errno != EINTR) {perror("readByteSerialPort"); break;}
        if (r > 0) nextState(p, byte);
    }
    alarm (0);
    return (p->state == STOP) ? 0:-1;
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    params = llParameters; 
    
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened\n", llParameters.serialPort);



    // Create SET array 
    unsigned char set[] = {FLAG, A_TX, C_SET, A_TX ^ C_SET, FLAG}; 

    setupAlarm(); 

    // No UA after all attempts, connection not established
    if (sendAndWait(set, SUPERVISION_FRAME_SIZE, A_TX, C_UA, NO_REJ) < 0){
        printf("No UA after %d attempts\n", llParameters.nRetransmissions+1);
        if (closeSerialPort() < 0) {perror("closeSerialPort");}
        return -1; 
    }

    //UA received, connectione stablished. exit w success
    printf("UA Received\n");

    txNs = 0; 

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{

    params = llParameters; 

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    setupAlarm();

    FrameParser p;
    do {
        if (readFrame(&p, A_TX, 0) < 0){
            if (closeSerialPort() < 0) {perror("closeSerialPort");}
            return -1;
        }
    } while (p.c != C_SET);
    printf("SET received\n");


    if (sendSupervision(A_TX, C_UA) <0){
        if(closeSerialPort()<0){perror("close Serial Port");}
        return -1; 
    }

    printf("%d bytes written to serial port\n", SUPERVISION_FRAME_SIZE);

    rxExpectedNs = 0; 
    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize) {
    if (buf == NULL || bufSize <=  0  || bufSize > MAX_PAYLOAD_SIZE){
        return -1;
    }
    unsigned char BCC2 = 0; 
    int i = 0; 
    while (i < bufSize){
        BCC2 = BCC2 ^ buf[i];
        i++; 
    }

    // F A C BCC (4) 
    // 2* (dados + BCC2)
    // F (1) 
    int maxBufSize = 4 + 2*(MAX_PAYLOAD_SIZE+1) +1;
    unsigned char stuffed[maxBufSize];
    
    stuffed[0] = FLAG;
    stuffed[1] = A_TX; 
    stuffed[2] = txNs ? C_ns1 : C_ns0; 
    stuffed[3] = stuffed[1] ^ stuffed[2];
    i = 4; int j = 0; 
    while (j < bufSize){
        if (buf[j] == 0x7D || buf[j] == 0x7E ){
            stuffed[i] = 0x7D; 
            i++; 
            stuffed[i] = buf[j] ^ ESC_XOR;
        }
        else{
            stuffed[i] = buf[j];
        }
        j++; 
        i++;
    }

    if (BCC2 == 0x7D || BCC2 == 0x7E ){
        stuffed[i] = 0x7D; 
        i++; 
        stuffed[i] = BCC2 ^ ESC_XOR;
    }
    else{
        stuffed[i] = BCC2;
    }
    i++; 
    stuffed[i] = FLAG; 
    i++; 

    stats.iFramesSent++;
    
    if (sendAndWait(stuffed, i, A_TX, txNs ? C_RR0 : C_RR1, txNs ? C_REJ1 : C_REJ0) < 0){
        return -1; 
    }
    txNs^= 1; 
    return bufSize;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    while (TRUE){
        FrameParser p; 
        if (readFrame(&p, A_TX, (params.nRetransmissions+1)*params.timeout) < 0) return -1;

        switch (p.c) {
            case C_ns0:
            case C_ns1: {
                int ns = (p.c == C_ns1) ? 1 : 0;

                unsigned char x = 0;
                for (int i = 0; i < p.dataLen; i++) x ^= p.data[i];
                int bcc2Ok = (p.dataLen >= 2) && (x == 0);

                if (ns != rxExpectedNs){
                    stats.duplicates++;
                    sendSupervision(A_TX, rxExpectedNs ? C_RR1 : C_RR0);
                    continue; 
                }

                if (!bcc2Ok){
                    stats.bcc2Errors++;
                    sendSupervision(A_TX, rxExpectedNs ? C_REJ1 : C_REJ0);
                    continue; 
                }

                for (int i = 0; i < p.dataLen-1; i++){packet[i] = p.data[i];}

                stats.iFramesReceived++;
                rxExpectedNs ^= 1; 
                sendSupervision(A_TX, rxExpectedNs ? C_RR1 : C_RR0);
                return p.dataLen-1;
            }
            case C_SET: {
                sendSupervision(A_TX, C_UA);
                continue;
            }

            case C_DISC: {
                return -1;
            }


            // RR/REJ/UA aren't meant for the Rx: ignore
            default:
                continue;
        }
    }
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
