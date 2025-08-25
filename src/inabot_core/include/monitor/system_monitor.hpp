#ifndef SYSTEM_MONITOR_HPP
#define SYSTEM_MONITOR_HPP

#include <string>
#include <vector>
#include <chrono>

class SystemMonitor
{
public:
    explicit SystemMonitor(const std::string &disk_device = "nvme0n1");
    void getSystemStatus();

private:
    struct CpuTimes
    {
        unsigned long long user = 0;
        unsigned long long nice = 0;
        unsigned long long system = 0;
        unsigned long long idle = 0;
        unsigned long long iowait = 0;
        unsigned long long irq = 0;
        unsigned long long softirq = 0;
    };

    struct DiskStats
    {
        unsigned long long sectors_read = 0;
        unsigned long long sectors_written = 0;
    };

    CpuTimes readCpuTimes() const;
    float calculateCpuUsage(const CpuTimes &prev, const CpuTimes &curr) const;

    float readCpuTemp() const;
    void readMemUsage(unsigned long &total, unsigned long &used) const;
    DiskStats readDiskStats(const std::string &device) const;

    void readTopProcesses(std::vector<std::string> &proc_names,
                          std::vector<float> &proc_usages,
                          const std::string &type) const;

private:
    CpuTimes prev_cpu_times_;
    bool first_run_ = true;

    DiskStats prev_disk_stats_;
    bool first_disk_run_ = true;

    std::string disk_device_;

    static constexpr unsigned int SECTOR_SIZE = 512;
    static constexpr bool USE_CPU_DISK = true;

    std::chrono::steady_clock::time_point high_cpu_start_time_;
    bool high_cpu_flag_ = false;
    static constexpr int HIGH_CPU_DURATION_SEC = 60;
};

#endif // SYSTEM_MONITOR_HPP
