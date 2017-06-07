#pragma once

#include <string>
#include <termios.h>
#include <unistd.h>

class SerialPort {

    private:
        int ReadBufferSize;
        int timeout;
        char terminationChar;
        bool termination;
        std::string id;
        int baudRate;
        int fd;
        struct termios tty;

    public:
		SerialPort(int baudRate, const std::string &id);
		void Init(int baudRate, const std::string &id);
		void SetReadBufferSize(int size);
		void SetTimeout(int timeout);
		void EnableTermination(char c);
		void Flush(void);
		void Write(const char *data, int length);
		int GetBytesReceived(void);
		int Read(char *data, int size);
		void WaitForData(void);
		void Reset(void);
		void Close(void);
};
