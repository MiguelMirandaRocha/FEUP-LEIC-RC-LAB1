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
#define A_TX 0x03
#define A_RX 0x01

// C Control
#define C_SET 0x03
#define C_UA 0x07


//states
typedef enum {
    START, 
    FLAG_RCV, 
    A_RCV, 
    C_RCV, 
    BCC_OK, 
    STOP
} State; 

int alarmEnabled = FALSE; 
int alarmCount = 0; 

static State nextState(State s, unsigned char byte, unsigned char expectedA, unsigned char expectedC){
    switch (s){
        case START:
            if (byte == FLAG) s = FLAG_RCV;
            break; 

        case FLAG_RCV:
            if (byte == expectedA) s = A_RCV; 
            else if (byte == FLAG) s = FLAG_RCV;
            else s= START; 
            break; 

        case A_RCV: 
            if (byte == expectedC) s = C_RCV; 
            else if (byte == FLAG) s = FLAG_RCV; 
            else s = START; 
            break; 

        case C_RCV: 
            if (byte == (expectedA ^ expectedC)) s = BCC_OK; 
            else if (byte == FLAG) s = FLAG_RCV; 
            else s = START; 
            break; 

        case BCC_OK: 
            if (byte == FLAG) s = STOP; 
            else s= START; 
            break; 
        case STOP: 
            break; 
    }
    return s; 
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
    

    State s = START; 
    alarmEnabled = FALSE; 
    alarmCount = 0; 
    while (alarmCount < llParameters.nRetransmissions && s != STOP){
        if (alarmEnabled == FALSE){
            writeBytesSerialPort(buf, SUPERVISION_FRAME_SIZE);
            alarm(llParameters.timeout);
            alarmEnabled = TRUE; 
            s = START; 
        }
        unsigned char byte; 
        if (readByteSerialPort(&byte) > 0){
            s = nextState(s,byte, A_TX, C_UA);
        }
    }
    alarm(0);

    // No UA after all attempts, connection not established
    if (s != STOP){
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

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

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
    State s = START;
    while (s != STOP){
        unsigned char byte;
        if (readByteSerialPort(&byte) > 0){
            s = nextState(s, byte, A_TX, C_SET);
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
    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

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
