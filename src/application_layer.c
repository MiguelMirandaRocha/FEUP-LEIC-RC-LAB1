// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>


#define PKT_START 1
#define PKT_DATA  2
#define PKT_END   3

#define T_FILE_SIZE 0
#define T_FILE_NAME 1

#define MAX_NAME_SIZE 255

// C L2 L1 + data must fit in one llSend
#define MAX_DATA_SIZE (MAX_PAYLOAD_SIZE - 3)


static int buildControlPacket(unsigned char *pkt, unsigned char c, unsigned int fileSize, const char *fileName){
    int i = 0;
    pkt[i++] = c;

    pkt[i++] = T_FILE_SIZE;
    pkt[i++] = 4;
    pkt[i++] = (fileSize >> 24) & 0xFF;
    pkt[i++] = (fileSize >> 16) & 0xFF;
    pkt[i++] = (fileSize >> 8) & 0xFF;
    pkt[i++] = fileSize & 0xFF;

    int nameLen = strlen(fileName);
    if (nameLen > MAX_NAME_SIZE) nameLen = MAX_NAME_SIZE;
    pkt[i++] = T_FILE_NAME;
    pkt[i++] = nameLen;
    memcpy(pkt + i, fileName, nameLen);
    i += nameLen;

    return i;
}

// Reads the parameters of a START or END packet of n bytes.
// Returns 0 on success, -1 if the packet is malformed or has no file size.
static int parseControlPacket(const unsigned char *pkt, int n, unsigned int *fileSize, char *fileName){
    int hasSize = FALSE;
    fileName[0] = '\0';

    int i = 1;
    while (i < n){
        if (i + 2 > n) return -1;
        unsigned char t = pkt[i];
        unsigned char l = pkt[i + 1];
        const unsigned char *v = pkt + i + 2;
        if (i + 2 + l > n) return -1;

        if (t == T_FILE_SIZE){
            *fileSize = 0;
            for (int k = 0; k < l; k++) *fileSize = (*fileSize << 8) | v[k];
            hasSize = TRUE;
        }
        else if (t == T_FILE_NAME){
            memcpy(fileName, v, l);
            fileName[l] = '\0';
        }

        i += 2 + l;
    }
    return hasSize ? 0 : -1;
}

////////////////////////////////////////////////
// TRANSMITTER
////////////////////////////////////////////////


static int sendPackets(FILE *file, unsigned int fileSize, const char *fileName){
    unsigned char pkt[MAX_PAYLOAD_SIZE];

    int len = buildControlPacket(pkt, PKT_START, fileSize, fileName);
    if (llSend(pkt, len) < 0) return -1;

    int k;
    while ((k = fread(pkt + 3, 1, MAX_DATA_SIZE, file)) > 0){
        pkt[0] = PKT_DATA;
        pkt[1] = k / 256;   // L2
        pkt[2] = k % 256;   // L1
        if (llSend(pkt, k + 3) < 0) return -1;
    }

    if (ferror(file)){
        perror("fread");
        return -1;
    }

    len = buildControlPacket(pkt, PKT_END, fileSize, fileName);
    if (llSend(pkt, len) < 0) return -1;

    return 0;
}

static void runTx(LinkLayer llParameters, const char *filename){
    FILE *file = fopen(filename, "rb");
    if (file == NULL) {perror(filename); return;}

    fseek(file, 0, SEEK_END);
    unsigned int fileSize = ftell(file);
    rewind(file);

    if (llOpenTx(llParameters) < 0){
        printf("Could not establish the connection\n");
        fclose(file);
        return;
    }

    if (sendPackets(file, fileSize, filename) < 0) printf("Transfer failed\n");
    else printf("File sent: %s (%u bytes)\n", filename, fileSize);

    llCloseTx();
    fclose(file);
}

////////////////////////////////////////////////
// RECEIVER
////////////////////////////////////////////////

static double nowSeconds(){
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

// Receives packets until END, writing the DATA to file.
// Prints the efficiency S = R / C, timed from START to END.
// Returns 0 if the whole file arrived, -1 otherwise.
static int receivePackets(FILE *file, int baudRate){
    unsigned char pkt[MAX_PAYLOAD_SIZE];
    unsigned int fileSize = 0, endSize = 0, received = 0;
    char fileName[MAX_NAME_SIZE + 1] = "", endName[MAX_NAME_SIZE + 1];
    int started = FALSE;
    double start = 0;

    while (TRUE){
        int n = llReceive(pkt);
        if (n < 0) return -1;

        switch (pkt[0]){
            case PKT_START:
                // A second START means the Tx restarted: start the file over
                if (started){
                    printf("New START, restarting the file\n");
                    rewind(file);
                    if (ftruncate(fileno(file), 0) < 0){
                        perror("ftruncate");
                        return -1;
                    }
                    received = 0;
                }
                started = TRUE;
                start = nowSeconds();

                if (parseControlPacket(pkt, n, &fileSize, fileName) < 0){
                    printf("Malformed START packet\n");
                    return -1;
                }
                printf("Receiving %s (%u bytes)\n", fileName, fileSize);
                break;

            case PKT_DATA: {
                int k = 256 * pkt[1] + pkt[2];
                if (n < 3 || k != n - 3){
                    printf("DATA packet size doesn't match its L2 L1\n");
                    return -1;
                }
                if (fwrite(pkt + 3, 1, k, file) != (size_t)k) {
                    perror("fwrite"); 
                    return -1;
                }
                received += k;
                break;
            }

            case PKT_END: {
                if (parseControlPacket(pkt, n, &endSize, endName) < 0 || endSize != fileSize || strcmp(endName, fileName) != 0){
                    printf("END packet doesn't match START\n");
                    return -1;
                }
                if (received != fileSize){
                    printf("Received %u of %u bytes\n", received, fileSize);
                    return -1;
                }

                // R counts only the file's bits, C is the baud rate
                double elapsed = nowSeconds() - start;
                double r = received * 8.0 / elapsed;
                printf("\n--- Efficiency ---\n");
                printf("Transfer time: %.3f s\n", elapsed);
                printf("Received bitrate (R): %.0f bit/s\n", r);
                printf("Link capacity (C): %d bit/s\n", baudRate);
                printf("Efficiency (S = R/C): %.3f\n\n", r / baudRate);
                return 0;
            }

            default:
                printf("Unknown packet type %d, ignored\n", pkt[0]);
        }
    }
}

static void runRx(LinkLayer llParameters, const char *filename){
    FILE *file = fopen(filename, "wb");
    if (file == NULL) {
        perror(filename); 
        return;
    }

    if (llOpenRx(llParameters) < 0){
        printf("Could not establish the connection\n");
        fclose(file);
        return;
    }

    if (receivePackets(file, llParameters.baudRate) < 0) printf("Transfer failed\n");
    else printf("File received: %s\n", filename);

    llCloseRx();
    fclose(file);
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

    if (strcmp(role, "tx") == 0) {
        runTx(llParameters, filename);
    }
    else if (strcmp(role, "rx") == 0) {
        runRx(llParameters, filename);
    }
    else printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
}
