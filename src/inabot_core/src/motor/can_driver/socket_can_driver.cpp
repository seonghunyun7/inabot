#include "socket_can_driver.hpp"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

//#include <utils/logger.h>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <cerrno>

namespace Inabot {

SocketCanDriver::SocketCanDriver()
    : socket_fd_(-1), is_open_(false) {}

SocketCanDriver::~SocketCanDriver() {
    close();
}

bool SocketCanDriver::initialize(const std::string& interface_name) {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (is_open_) {
        std::cout << "[Warning] CAN 인터페이스가 이미 열려 있습니다." << std::endl;
        return false;
    }

#if check_bitrate
    checkBitrateStatus(interface_name);
#endif

    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cout << "[Error] socket open error: " << strerror(errno) << std::endl;
        return false;
    }

#if set_buffer
    int rcvbuf_size = BUFF_SIZE;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
        std::cout << "[Warning] setsockopt SO_RCVBUF failed" << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
#endif

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cout << "[Error] ioctl() 실패: " << strerror(errno) << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cout << "[Error] bind() 실패: " << strerror(errno) << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    interface_name_ = interface_name;
    is_open_ = true;
    return true;
}

void SocketCanDriver::close() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (is_open_ && socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        is_open_ = false;
    }
}

bool SocketCanDriver::checkBitrateStatus(const std::string& interface_name) {
    std::string cmd = "ip -details link show " + interface_name + " 2>&1";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        std::cout << "[Warning] 'ip link show' 명령 실행 실패 - 비트레이트 상태 확인 불가" << std::endl;
        return false;
    }

    char buffer[512];
    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }
    pclose(pipe);

    bool has_bitrate = output.find("bitrate") != std::string::npos;
    bool is_down = output.find("DOWN") != std::string::npos;

    if (!has_bitrate || is_down) {
        std::cout << "[Warning] CAN 인터페이스 DOWN 또는 비트레이트 설정 안됨" << std::endl;
        std::cout << "    -> 'sudo ip link set " << interface_name
                  << " type can bitrate 500000 && sudo ip link set " << interface_name << " up'" << std::endl;
        return false;
    }

    return true;
}

bool SocketCanDriver::sendFrame(uint32_t can_id, const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!is_open_) {
        std::cout << "[Warning] CAN 인터페이스가 열려 있지 않습니다." << std::endl;
        return false;
    }

    if (data.size() > 8) {
        std::cout << "[Error] CAN 프레임 데이터 길이 초과: " << data.size() << std::endl;
        return false;
    }

    ::can_frame frame {};
    frame.can_id = can_id;
    frame.can_dlc = static_cast<__u8>(data.size());
    std::memcpy(frame.data, data.data(), frame.can_dlc);

    int nbytes = write(socket_fd_, &frame, sizeof(::can_frame));
    if (nbytes != sizeof(::can_frame)) {
        std::cout << "[Error] write() 실패: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "[Info] 전송 완료 - CAN ID: 0x"
              << std::hex << can_id << std::dec
              << ", 데이터 길이: " << data.size() << std::endl;

    return true;
}

::can_frame SocketCanDriver::readFrame() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    ::can_frame frame {};

    if (!is_open_) {
        std::cout << "[Warning] CAN 인터페이스가 열려 있지 않습니다." << std::endl;
        return frame;
    }

    ssize_t nbytes = read(socket_fd_, &frame, sizeof(::can_frame));
    if (nbytes < 0) {
        std::cout << "[Error] read() 실패: " << strerror(errno) << std::endl;
    } else if (nbytes == 0) {
        std::cout << "[Warning] 소켓 종료됨 (0 바이트 읽음)" << std::endl;
    } else if (nbytes < static_cast<ssize_t>(sizeof(::can_frame))) {
        std::cout << "[Warning] 짧은 프레임 수신: " << nbytes << " 바이트" << std::endl;
    } else {
#if __LOG__
std::ostringstream oss;
        oss << "[Info] 수신 프레임 - CAN ID: 0x"
            << std::hex << std::setw(8) << std::setfill('0') << frame.can_id
            << ", DLC: " << std::dec << static_cast<int>(frame.can_dlc) << ", Data: ";
        for (int i = 0; i < frame.can_dlc; ++i) {
            oss << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << oss.str() << std::endl;
#endif
    }

    return frame;
}

//PEAK
TPCANMsg_ SocketCanDriver::readPeakFrame() {
    TPCANMsg_ empty_msg{};
    empty_msg.ID = 0;
    empty_msg.MSGTYPE = 0;
    empty_msg.LEN = 0;
    for (int i = 0; i < 8; ++i)
        empty_msg.DATA[i] = 0;
    return empty_msg;
}


} // namespace Inabot
