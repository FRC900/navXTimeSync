#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

using namespace std;

class SerialPort {


    private:
        int ReadBufferSize;
        int timeout;
        char terminationChar;
        bool termination;
        string id;
        int baudRate;
        int fd;
        struct termios tty;

    public:

    SerialPort(int baudRate, string id) {


        int USB = open(id.c_str(), O_RDWR| O_NOCTTY);
        
        memset(&tty, 0, sizeof(tty));
        this->baudRate = baudRate;
        this->id = id;
        

        cfsetospeed(&tty, (speed_t)baudRate);
        cfsetispeed(&tty, (speed_t)baudRate);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 5;

        tty.c_cflag |= CREAD | CLOCAL;

        cfmakeraw(&tty);
        tcflush(USB, TCIFLUSH);
        if(tcsetattr(USB,TCSANOW,&tty) != 0) cout << "Failed to initialize serial.";

        this->fd = USB;

    }

    void SetReadBufferSize(int size) {
        this->ReadBufferSize = size;
        memset(&tty, 0, size);
    }

    void SetTimeout(int timeout) {
        this->timeout = timeout;
        tty.c_cc[VTIME] = timeout;
        tcflush(this->fd, TCIFLUSH);
        if(tcsetattr(this->fd,TCSANOW,&tty) != 0) cout << "Failed to initialize serial.";        
    }


    void EnableTermination(char c) {
        this->termination = true;
        this->terminationChar = c;
    }


    void Flush() {
        tcflush(this->fd, TCIFLUSH);
    }

    void Write(const std::string &data, int length) {
        int n_written = 0, spot = 0;

        do {
            n_written = write( this->fd, &data[spot], 1 );
            spot += n_written;
        } while (data[spot-1] != '\r' && n_written > 0);        
    }

    int GetBytesReceived() {
        int bytes_avail;
        ioctl(this->fd, FIONREAD, &bytes_avail);
        return bytes_avail;
    }

    int Read(char *data, int size) {
        int n = 0, loc = 0;
        char buf = '\0';

        memset(data, '\0', this->ReadBufferSize);

        do {
            n = read(this->fd, &buf, 1 );
            sprintf( &data[loc], "%c", buf );
            loc += n;
        } while( buf != '\r' && n > 0);

        if (n < 0) {
            std::cout << "Error reading: " << strerror(errno) << std::endl;
        }
        else if (n == 0) {
            std::cout << "Read nothing!" << std::endl;
        }        
        else {
            std::cout << "Response: " << data  << std::endl;
        }
        return 0;
    }

    void Reset() {
        tcflush(this->fd, TCIOFLUSH);
    }

};
