#pragma once

#include <csignal>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/stacktrace.hpp>

class CrashHandler {
public:
    static void setup();
private:
    static void handleSignal(int sig);
    static void logStackTrace(int sig);
};
