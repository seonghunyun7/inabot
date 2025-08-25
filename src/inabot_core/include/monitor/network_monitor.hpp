#ifndef NETWORK_MONITOR_HPP
#define NETWORK_MONITOR_HPP

#include <string>
#include <vector>
#include <future>

class NetworkMonitor
{
public:
    explicit NetworkMonitor(const std::string &bridge_ip, const std::string &factory_ip);

    // 네트워크 상태 체크 함수 (ping, route, interface 등)
    void checkNetworkStatus();

private:
    // 비동기 ping 함수
    bool pingDevice(const std::string &ip);

    // 비동기 라우트 정보 조회
    std::string checkRouteInfo();

    // 비동기 인터페이스 상태 조회
    std::string checkInterfaceStatus();

    // future 결과를 timeout까지 기다리는 헬퍼 함수들
    bool waitFutureWithTimeout(std::future<bool> &future, bool &result, std::chrono::milliseconds timeout);
    std::string waitFutureWithTimeout(std::future<std::string> &future, std::string &result, std::chrono::milliseconds timeout);

private:
    std::string bridge_router_ip_;
    std::string factory_router_ip_;
};

#endif // NETWORK_MONITOR_HPP
