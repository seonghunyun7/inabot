#ifndef _Y3Space_SERIAL_INTERFACE_H
#define _Y3Space_SERIAL_INTERFACE_H

#include <malloc.h>
//#include <libserial/serial.h> //libserial https://github.com/crayzeewulf/libserial LibSerial::SerialPort
#include <serial/serial.h> //serial by wjwwood serial::Serial	https://github.com/wjwwood/serial
#include <iostream>
#include <memory>         

using namespace serial;

class SerialInterface
{
private:
    using SerialPtr = std::unique_ptr<serial::Serial>;

public:
    SerialInterface(std::string port, int baudrate, int timeout);
    virtual ~SerialInterface();
    virtual bool serialConnect();
    virtual void serialDisconnect();

    virtual void serialWrite(uint8_t *buf, size_t len);
    virtual void serialWriteString(const std::string& str);
    virtual uint8_t serialReadByte();
    virtual std::string serialReadLine();
    virtual uint8_t* serialReadBytes(size_t nbytes);
    virtual size_t available();
    const int& getBaudRate() { return m_baudrate; }
    const std::string& getSerialPort() { return m_port; }

private:
    std::string m_port;
    int m_baudrate;
    int m_timeout;
    SerialPtr m_connection;
};
#endif //_Y3Space_SERIAL_INTERFACE_H
