#include "imu/SerialInterface.h"

SerialInterface::SerialInterface(std::string port, int baudrate, int timeout)
{
    m_port = port;
    m_baudrate = baudrate;
    m_timeout = timeout;
}

SerialInterface::~SerialInterface()
{
    if (m_connection != NULL)
    {
      	if(m_connection->isOpen())
      	{
			std::cout << "Closing the Serial Port" << std::endl;
        	m_connection->close();
      	}
    }
}

bool SerialInterface::serialConnect()
{
    try
    {
        m_connection.reset(new Serial(m_port, (uint32_t)m_baudrate, Timeout::simpleTimeout(m_timeout)));
    }
    catch (IOException &e)
    {
        std::string ioerror = e.what();
        std::cerr << "Unable to connect port: " << m_port << std::endl;
        std::cerr << "Is the device plugged in? Is the serial port open?" << std::endl;
        std::cerr << "Error: " << ioerror << std::endl;
        std::cerr << "Detailed error information: " << e.what() << std::endl;
        return false;
    }

    // 연결이 성공했는지 확인
    if (m_connection && m_connection->isOpen())
    {
        std::cout << "Connection Established with Port: " << m_port
                  << " with baudrate: " << m_baudrate << std::endl;
        return true;
    } 
    else 
    {
        std::cerr << "Failed to establish connection on port: " << m_port << std::endl;
        return false;
    }
}

void SerialInterface::serialDisconnect()
{
    if (m_connection)
    {
        if (m_connection->isOpen())
        {
            std::cout << "Closing serial port: " << m_port << std::endl;
            m_connection->close();
        }
        m_connection.reset();
    }
    else
    {
        std::cout << "Serial connection already null." << std::endl;
    }
}

void SerialInterface::serialWrite(uint8_t *buf, size_t len)
{
    size_t written = this->m_connection->write(buf, len);
    if (written != len)
    {
        std::cout << "Len: " << len << " .. Written: " << written << std::endl;
    }
}

void SerialInterface::serialWriteString(const std::string& str)
{
    size_t written = this->m_connection->write(str);
    if (written != str.size())
    {
        std::cerr << "Error: Failed to write the full string to the serial interface. Expected to write "
                  << str.size() << " bytes, but wrote " << written << " bytes." << std::endl;
    }
}
 
uint8_t SerialInterface::serialReadByte()
{
    uint8_t buf = 0;
    if (this->m_connection->available() > 0)
    {
        size_t bytes = this->m_connection->read(&buf, 1);
        if (bytes != 1)
        {
            std::cerr << "Warning: Unable to read byte from serial connection." << std::endl;
        }
    }
    return buf;
}

std::string SerialInterface::serialReadLine()
{
    try {
        return this->m_connection->readline();
    }
    catch (const serial::IOException& e) {
        std::cerr << "[SerialInterface] serial::IOException: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialInterface] std::exception: " << e.what() << std::endl;
    }

    return "";
}

uint8_t* SerialInterface::serialReadBytes(size_t nbytes)
{
    uint8_t* buf = (uint8_t*)malloc(sizeof(uint8_t) * nbytes);
    if (!buf)
    {
        std::cerr << "Error: Memory allocation failed for serialReadBytes." << std::endl;
        return nullptr;
    }

    if (this->m_connection->available() > 0)
    {
        size_t bytes = this->m_connection->read(buf, nbytes);
        if (bytes != nbytes)
        {
            std::cerr << "Warning: Unable to read " << nbytes << " bytes from serial connection." << std::endl;
        }
    }
    return buf;
}

size_t SerialInterface::available()
{
    return this->m_connection->available();
}
