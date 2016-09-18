#include <unistd.h>
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <time.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/un.h>



// Structure definitions

// Public function prototypes

// Private function prototypes

static int set_interface_attribs (int fd, int speed, int parity);



static int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays

    // For details about blocking in non canonical (binary mode)
    // check out http://unixwiz.net/techtips/termios-vmin-vtime.html

    tty.c_cc[VMIN]  = 0;            // no need to wait for any bytes in the buffer
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}


int main(void)
{
    int cnt = 0;
    int n = 0;
    uint8_t ubxBuf[150];
    FILE *ubxPtr;
    int k = 0;

    // Commands taken from u-center, killNema is to silence the nema messages
    uint8_t killNema_1[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F};
    uint8_t killNema_2[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
    uint8_t killNema_3[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
    uint8_t killNema_4[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    uint8_t killNema_5[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
    uint8_t killNema_6[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};
    uint8_t killNema_7[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F};

    uint8_t reqAidAlm[8] = { 0xb5, 0x62, 0x0b, 0x30, 0x00, 0x00, 0x3b, 0xBC};

    //uint8_t reqAidHui[8] = { 0xb5, 0x62, 0x0b, 0x02, 0x00, 0x00, 0x0D, 0x32};
    //uint8_t reqAidEph[8] = { 0xb5, 0x62, 0x0b, 0x31, 0x00, 0x00, 0x3C, 0xbf};
    //uint8_t reqAidAlm[9] = { 0xb5, 0x62, 0x0b, 0x30, 0x01, 0x00, 0x0b, 0x47, 0x05}; // single satellite
    //uint8_t reqAidIni[8] = { 0xb5, 0x62, 0x0b, 0x01, 0x00, 0x00, 0x0c, 0x2f};
    //uint8_t reqNavLlh[8] = { 0xb5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0a};


    char *portname = "/dev/ttyACM0";

    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);

    set_interface_attribs (fd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)

    usleep(10);

    memset(ubxBuf, 0, sizeof(uint8_t)*150);

    ubxPtr = fopen("rawData.ubx", "wb");



    // Quiet down Nema Logs
    n = write(fd, killNema_1, sizeof(uint8_t) * 11);
    if(n == sizeof(uint8_t) *11){ printf("kill command 1 success\n"); }
    usleep(500);

    n = write(fd, killNema_2, sizeof(uint8_t) * 11);
    if(n == sizeof(uint8_t) *11){ printf("kill command 2 success\n"); }
    usleep(500);

    n = write(fd, killNema_3, sizeof(uint8_t) * 11);
    if(n == sizeof(uint8_t) *11){ printf("kill command 3 success\n"); }
    usleep(500);

    n = write(fd, killNema_4, sizeof(uint8_t) * 11);
    if(n == sizeof(uint8_t) *11){ printf("kill command 4 success\n"); }
    usleep(500);

    n = write(fd, killNema_5, sizeof(uint8_t) * 11);
    if(n == sizeof(uint8_t) *11){ printf("kill command 5 success\n"); }
    usleep(500);

    n = write(fd, killNema_6, sizeof(uint8_t) * 11);
    if(n == sizeof(uint8_t) *11){ printf("kill command 6 success\n"); }
    usleep(500);

    n = write(fd, killNema_7, sizeof(uint8_t) * 11);
    if(n == sizeof(uint8_t) *11){ printf("kill command 7 success\n"); }
    usleep(500);

    sleep(1);
    tcflush(fd, TCIFLUSH); /* Discards old data in the rx buffer */
    usleep(500);


    while(1)
    {
        
        // Read incoming data from Lea-6T
        n = read(fd, (uint8_t *)ubxBuf, sizeof(uint8_t)*6);
        if( n > 0){
            printf("incoming %d bytes: ", n);
            for(int ii=0; ii < n; ii++){ 
                printf("%02X", ubxBuf[ii]);
            }
            printf("\n");
            k = fwrite(ubxBuf, 1, sizeof(uint8_t)*n, ubxPtr);
            //printf("wrote %d bytes\n", k);
            //fflush(ubxPtr);
            memset(ubxBuf, 0, sizeof(uint8_t) * 150);
        }else{
            printf("cnt: %d\n",cnt);
            cnt++;
        }

        // Ask for Almanac data once
        if(cnt == 15){
            printf("sending aid alm\n");
            write(fd,  reqAidAlm, sizeof(uint8_t)*8);
            usleep(50);
            cnt++;
        }

        if(cnt > 24){break;}

    }

    fclose(ubxPtr);
    printf("exiting now...\n");
    
    return 0;

}
