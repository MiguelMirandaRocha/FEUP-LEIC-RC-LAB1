// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

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

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and send a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

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


    int bytes = writeBytesSerialPort(buf, SUPERVISION_FRAME_SIZE);
    printf("%d bytes written to serial port\n", bytes);

    // Wait until all bytes have been written to the serial port
    sleep(1);

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

    // Read from serial port until the 'z' char is received.

    // NOTE: This while() cycle is a simple example showing how to read from the serial port.
    // It must be changed in order to respect the specifications of the protocol indicated in the Lab guide.

    // TODO: Save the received bytes in a buffer array and print it at the end of the program.
    State s = START; 

    while (s != STOP)
    {
        unsigned char byte;

        // If read was unsuccessful, retry
        if (readByteSerialPort(&byte) <= 0){
            continue; 
        }

        switch (s){
            case START:
                if (byte == FLAG) s = FLAG_RCV;
                break; 
            
            case FLAG_RCV:
                if (byte == A_TX) s = A_RCV; 
                else if (byte == FLAG) s = FLAG_RCV;
                else s= START; 
                break; 

            case A_RCV: 
                if (byte == C_SET) s = C_RCV; 
                else if (byte == FLAG) s = FLAG_RCV; 
                else s = START; 
                break; 

            case C_RCV: 
                if (byte == (A_TX ^ C_SET)) s = BCC_OK; 
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
    }
    printf("SET received\n");

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
