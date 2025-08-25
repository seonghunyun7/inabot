#include "network_monitor.hpp"

#include <iostream>
#include <array>
#include <cstdio>
#include <chrono>
//#include <utils/logger.h>

//bridge_router_ip는 물류 로봇이 직접 연결된 공유기 IP (즉, 로봇이 연결된 네트워크의 게이트웨이 IP)
//factory_router_ip는 그 공유기가 연결된 공장 내부 라우터 IP (즉, 공유기에서 바라보는 상위 라우터 IP)

NetworkMonitor::NetworkMonitor(const std::string &bridge_ip, const std::string &factory_ip)
    : bridge_router_ip_(bridge_ip), factory_router_ip_(factory_ip)
{
}

void NetworkMonitor::checkNetworkStatus()
{
    auto bridge_ping_future = std::async(std::launch::async, &NetworkMonitor::pingDevice, this, bridge_router_ip_);
    auto factory_ping_future = std::async(std::launch::async, &NetworkMonitor::pingDevice, this, factory_router_ip_);
    auto route_future = std::async(std::launch::async, &NetworkMonitor::checkRouteInfo, this);
    auto interface_future = std::async(std::launch::async, &NetworkMonitor::checkInterfaceStatus, this);

    bool bridge_online = false;
    bool factory_online = false;
    std::string route_info = "timeout";
    std::string interface_info = "timeout";

    std::chrono::milliseconds timeout(2000);

    if (!waitFutureWithTimeout(bridge_ping_future, bridge_online, timeout)) {
        std::cout << "[WARNING] Bridge router ping timeout" << std::endl;
    }
    if (!waitFutureWithTimeout(factory_ping_future, factory_online, timeout)) {
        std::cout << "[WARNING] Factory router ping timeout" << std::endl;
    }
    if (waitFutureWithTimeout(route_future, route_info, timeout) != "ok") {
        route_info = "Route Info Timeout";
        std::cout << "[WARNING] Route info fetch timeout" << std::endl;
    }
    if (waitFutureWithTimeout(interface_future, interface_info, timeout) != "ok") {
        interface_info = "Interface Info Timeout";
        std::cout << "[WARNING] Interface info fetch timeout" << std::endl;
    }

    std::cout << "[INFO] Bridge Router Online: " << (bridge_online ? "Yes" : "No") << std::endl;
    std::cout << "[INFO] Factory Router Online: " << (factory_online ? "Yes" : "No") << std::endl;
    std::cout << "[INFO] Route Info: " << route_info << std::endl; //debug
    std::cout << "[INFO] Interface Info: " << interface_info << std::endl;
}

bool NetworkMonitor::pingDevice(const std::string &ip)
{
    std::string cmd = "ping -c 1 -W 1 " + ip + " > /dev/null 2>&1";
    int ret = std::system(cmd.c_str());
    return (ret == 0);
}

std::string NetworkMonitor::checkRouteInfo()
{
    std::array<char, 128> buffer;
    std::string result;
    FILE *pipe = popen("ip route", "r");
    if (!pipe)
        return "Failed to get routing info.";
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result += buffer.data();
    }
    pclose(pipe);
    return result;
}

std::string NetworkMonitor::checkInterfaceStatus()
{
    std::array<char, 128> buffer;
    std::string result;
    FILE *pipe = popen("ip addr", "r");
    if (!pipe)
        return "Failed to get interface info.";
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result += buffer.data();
    }
    pclose(pipe);
    return result;
}

bool NetworkMonitor::waitFutureWithTimeout(std::future<bool> &future, bool &result, std::chrono::milliseconds timeout)
{
    if (future.wait_for(timeout) == std::future_status::ready) {
        result = future.get();
        return true;
    }
    return false;
}

std::string NetworkMonitor::waitFutureWithTimeout(std::future<std::string> &future, std::string &result, std::chrono::milliseconds timeout)
{
    if (future.wait_for(timeout) == std::future_status::ready) {
        result = future.get();
        return "ok";
    }
    return "timeout";
}
