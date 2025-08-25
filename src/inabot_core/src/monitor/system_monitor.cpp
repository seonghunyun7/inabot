#include "monitor/system_monitor.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <array>
#include <algorithm>
#include <cstdlib>
#include <iomanip>  // std::setprecision
//#include <utils/logger.h>

SystemMonitor::SystemMonitor(const std::string &disk_device)
     : disk_device_(disk_device), first_run_(true), first_disk_run_(true)
{
}

SystemMonitor::CpuTimes SystemMonitor::readCpuTimes() const
{
    std::ifstream file("/proc/stat");
    CpuTimes times;

    if (!file.is_open())
        return times;

    std::string line;
    std::getline(file, line);

    std::istringstream ss(line);
    std::string cpu_label;
    ss >> cpu_label;
    ss >> times.user >> times.nice >> times.system >> times.idle
       >> times.iowait >> times.irq >> times.softirq;

    return times;
}

float SystemMonitor::calculateCpuUsage(const CpuTimes &prev, const CpuTimes &curr) const
{
    unsigned long long prev_idle = prev.idle + prev.iowait;
    unsigned long long curr_idle = curr.idle + curr.iowait;

    unsigned long long prev_non_idle = prev.user + prev.nice + prev.system + prev.irq + prev.softirq;
    unsigned long long curr_non_idle = curr.user + curr.nice + curr.system + curr.irq + curr.softirq;

    unsigned long long prev_total = prev_idle + prev_non_idle;
    unsigned long long curr_total = curr_idle + curr_non_idle;

    unsigned long long total_diff = curr_total - prev_total;
    unsigned long long idle_diff = curr_idle - prev_idle;

    if (total_diff == 0) return 0.0f;

    float cpu_percentage = static_cast<float>(total_diff - idle_diff) / total_diff * 100.0f;

    if (cpu_percentage < 0.0f) cpu_percentage = 0.0f;
    if (cpu_percentage > 100.0f) cpu_percentage = 100.0f;

    return cpu_percentage;
}

float SystemMonitor::readCpuTemp() const
{
    // CPU 온도 읽기 - 시스템마다 경로 다름. 여기서는 예시로 /sys/class/thermal/thermal_zone0/temp 사용
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open())
        return 0.0f;

    int temp_milli = 0;
    file >> temp_milli;
    return static_cast<float>(temp_milli) / 1000.0f;
}

void SystemMonitor::readMemUsage(unsigned long &total, unsigned long &used) const
{
    std::ifstream file("/proc/meminfo");
    total = 0;
    unsigned long free = 0, buffers = 0, cached = 0;
    if (!file.is_open())
        return;

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string key;
        unsigned long value;
        std::string unit;
        ss >> key >> value >> unit;
        if (key == "MemTotal:")
            total = value;
        else if (key == "MemFree:")
            free = value;
        else if (key == "Buffers:")
            buffers = value;
        else if (key == "Cached:")
            cached = value;
    }
    used = total - free - buffers - cached;
}

SystemMonitor::DiskStats SystemMonitor::readDiskStats(const std::string &device) const
{
    DiskStats stats{};
    std::ifstream file("/proc/diskstats");
    if (!file.is_open())
        return stats;

    std::string line;
    while (std::getline(file, line))
    {
        if (line.find(device) != std::string::npos)
        {
            std::istringstream ss(line);
            // /proc/diskstats 포맷:
            // major minor device_name reads completed sectors_read ... writes completed sectors_written ...
            // 우리가 필요한 건 sectors_read(5번째 필드), sectors_written(9번째 필드)
            // 필드 번호는 0부터 카운트
            std::string major, minor, devname;
            unsigned long long reads_completed, reads_merged, sectors_read;
            unsigned long long time_reading, writes_completed, writes_merged, sectors_written;

            ss >> major >> minor >> devname
               >> reads_completed >> reads_merged >> sectors_read >> time_reading
               >> writes_completed >> writes_merged >> sectors_written;

            stats.sectors_read = sectors_read;
            stats.sectors_written = sectors_written;
            break;
        }
    }
    return stats;
}

void SystemMonitor::readTopProcesses(std::vector<std::string> &proc_names,
                                         std::vector<float> &proc_usages,
                                         const std::string &type) const
{
    // top 명령어를 이용하여 상위 5개 프로세스 이름과 CPU 또는 MEM 사용률을 가져온다.
    // type은 "cpu" 또는 "mem" 중 하나
    proc_names.clear();
    proc_usages.clear();

    std::string cmd = "top -b -n 1 -o %";
    cmd += (type == "cpu") ? "CPU" : "MEM";

    // top 출력에서 필요한 부분만 추출
    std::string line;
    std::array<char, 512> buffer;

    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
        return;

    bool process_section = false;
    int count = 0;

    while (fgets(buffer.data(), (int)buffer.size(), pipe) != nullptr)
    {
        line = buffer.data();
        // 프로세스 리스트 시작부분 인식 (COMMAND로 시작)
        if (line.find("COMMAND") != std::string::npos)
        {
            process_section = true;
            continue;
        }
        if (process_section && count < 5)
        {
            std::istringstream ss(line);
            std::string pid, user, pr, ni, virt, res, shr, s, cpu_perc_str, mem_perc_str, time, command;

            ss >> pid >> user >> pr >> ni >> virt >> res >> shr >> s
               >> cpu_perc_str >> mem_perc_str >> time;

            std::getline(ss, command); // command는 나머지 전체

            // 앞뒤 공백 제거
            command.erase(command.find_last_not_of(" \n\r\t")+1);
            command.erase(0, command.find_first_not_of(" \n\r\t"));

            float usage = 0.0f;
            if (type == "cpu")
                usage = std::stof(cpu_perc_str);
            else
                usage = std::stof(mem_perc_str);

            proc_names.push_back(command);
            proc_usages.push_back(usage);
            ++count;
        }
    }
    pclose(pipe);
}

void SystemMonitor::getSystemStatus() 
{
    CpuTimes current_cpu = readCpuTimes();
    float cpu_use = 0.0f;
    if (first_run_)
    {
        prev_cpu_times_ = current_cpu;
        first_run_ = false;
        cpu_use = 0.0f;
    }
    else
    {
        cpu_use = calculateCpuUsage(prev_cpu_times_, current_cpu);
        prev_cpu_times_ = current_cpu;
    }

    float cpu_temp = readCpuTemp();

    unsigned long mem_total_kb = 0, mem_used_kb = 0;
    readMemUsage(mem_total_kb, mem_used_kb);
    float mem_total = static_cast<float>(mem_total_kb) / 1024.0f;
    float mem_used = static_cast<float>(mem_used_kb) / 1024.0f;

    float memory_usage_ratio = 0.0f;
    if (mem_total_kb > 0)
        memory_usage_ratio = static_cast<float>(mem_used_kb) / mem_total_kb;

    std::string disk_read_str = "0";
    std::string disk_write_str = "0";
    if (USE_CPU_DISK)
    {
        DiskStats current_disk = readDiskStats(disk_device_);
        if (first_disk_run_)
        {
            prev_disk_stats_ = current_disk;
            first_disk_run_ = false;
        }
        else
        {
            auto read_diff = (current_disk.sectors_read - prev_disk_stats_.sectors_read) * SECTOR_SIZE;
            auto write_diff = (current_disk.sectors_written - prev_disk_stats_.sectors_written) * SECTOR_SIZE;

            float read_kb = static_cast<float>(read_diff) / 1024.0f;
            float write_kb = static_cast<float>(write_diff) / 1024.0f;

            std::ostringstream oss_read, oss_write;
            oss_read << std::fixed << std::setprecision(2) << read_kb << " KB/s";
            oss_write << std::fixed << std::setprecision(2) << write_kb << " KB/s";

            disk_read_str = oss_read.str();
            disk_write_str = oss_write.str();

            prev_disk_stats_ = current_disk;
        }
    }

    std::vector<std::string> cpu_procs;
    std::vector<float> cpu_usages;
    readTopProcesses(cpu_procs, cpu_usages, "cpu");

    std::vector<std::string> mem_procs;
    std::vector<float> mem_usages;
    readTopProcesses(mem_procs, mem_usages, "mem");

    #if __LOG__
    std::cout << "[INFO] ====== System Status ======" << std::endl;
    std::cout << "[INFO] CPU Usage    : " << std::fixed << std::setprecision(2) << cpu_use << " %" << std::endl;
    std::cout << "[INFO] CPU Temp     : " << std::fixed << std::setprecision(1) << cpu_temp << " °C" << std::endl;
    std::cout << "[INFO] Memory Usage : " << mem_used << " MB / " << mem_total << " MB" << std::endl;
    // std::cout << "[INFO] Disk Read    : " << disk_read_str << std::endl;
    // std::cout << "[INFO] Disk Write   : " << disk_write_str << std::endl;

    std::cout << "[INFO] Top 5 CPU Processes:" << std::endl;
    for (size_t i = 0; i < cpu_procs.size(); ++i)
        std::cout << "  " << cpu_procs[i] << " : " << cpu_usages[i] << " %" << std::endl;
    // std::cout << "[INFO] Top 5 Memory Processes:" << std::endl;
    // for (size_t i = 0; i < mem_procs.size(); ++i)
    //     std::cout << "  " << mem_procs[i] << " : " << mem_usages[i] << " %" << std::endl;
    std::cout << "[INFO] ============================" << std::endl;
    #endif
    // CPU 사용량 경고
    using namespace std::chrono;
    auto now = steady_clock::now();
    if (cpu_use > 80.0f)
    {
        if (!high_cpu_flag_)
        {
            high_cpu_flag_ = true;
            high_cpu_start_time_ = now;
        }
        else
        {
            auto duration = duration_cast<seconds>(now - high_cpu_start_time_).count();
            if (duration >= 60)
            {
                std::cout << "[WARNING] High CPU usage sustained over 60 seconds: " << cpu_use << " %" << std::endl;
            }
        }
    }
    else
    {
        high_cpu_flag_ = false;
    }

    // CPU 온도 경고
    if (cpu_temp > 70.0f)
    {
        std::cout << "[WARNING] High CPU temperature detected: " << cpu_temp << " °C" << std::endl;
    }

    // 메모리 사용량 경고
    if (memory_usage_ratio > 0.8f)
    {
        std::cout << "[WARNING] High Memory usage: " << (memory_usage_ratio * 100.0f) << " %" << std::endl;
    }

    // 프로세스별 CPU 50% 이상 점유 경고
    for (size_t i = 0; i < cpu_procs.size(); ++i)
    {
        if (cpu_usages[i] > 50.0f)
        {
            std::cout << "[WARNING] Process " << cpu_procs[i] << " is using " << cpu_usages[i] << " % CPU" << std::endl;
        }
    }
}