#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace ipn_mpc::logging {

enum class Level : int {
    debug = 0,
    info = 1,
    warning = 2,
    error = 3,
    off = 4,
};

inline constexpr std::string_view name(Level level) noexcept {
    switch (level) {
    case Level::debug:
        return "DEBUG";
    case Level::info:
        return "INFO";
    case Level::warning:
        return "WARN";
    case Level::error:
        return "ERROR";
    case Level::off:
        return "OFF";
    }
    return "UNKNOWN";
}

inline Level parseLevel(const char* value) noexcept {
    if (value == nullptr) {
        return Level::info;
    }

    const std::string_view text(value);
    if (text == "debug" || text == "DEBUG") {
        return Level::debug;
    }
    if (text == "warning" || text == "WARN" || text == "WARNING") {
        return Level::warning;
    }
    if (text == "error" || text == "ERROR") {
        return Level::error;
    }
    if (text == "off" || text == "OFF") {
        return Level::off;
    }
    return Level::info;
}

inline Level configuredLevel() noexcept {
    static const Level level = parseLevel(std::getenv("IPN_MPC_LOG_LEVEL"));
    return level;
}

inline bool enabled(Level level) noexcept {
    return level >= configuredLevel() && configuredLevel() != Level::off;
}

inline std::mutex& outputMutex() {
    static std::mutex mutex;
    return mutex;
}

inline std::string_view filename(std::string_view path) noexcept {
    const auto separator = path.find_last_of("/\\");
    return separator == std::string_view::npos ? path : path.substr(separator + 1);
}

class LogLine {
  public:
    LogLine(Level level, const char* file, int line)
        : level_(level), file_(filename(file)), line_(line), enabled_(enabled(level)) {}

    ~LogLine() {
        if (!enabled_) {
            return;
        }

        const auto now = std::chrono::system_clock::now();
        const auto milliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        const std::time_t time = std::chrono::system_clock::to_time_t(now);
        std::tm local_time{};
#if defined(_WIN32)
        localtime_s(&local_time, &time);
#else
        localtime_r(&time, &local_time);
#endif

        std::lock_guard<std::mutex> lock(outputMutex());
        std::ostream& output = level_ >= Level::warning ? std::cerr : std::clog;
        output << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0')
               << std::setw(3) << milliseconds.count() << " [" << name(level_) << "] [" << file_
               << ':' << line_ << "] " << stream_.str() << '\n';
    }

    template <typename Value> LogLine& operator<<(Value&& value) {
        if (enabled_) {
            stream_ << std::forward<Value>(value);
        }
        return *this;
    }

    using Manipulator = std::ostream& (*)(std::ostream&);
    LogLine& operator<<(Manipulator manipulator) {
        if (enabled_) {
            manipulator(stream_);
        }
        return *this;
    }

  private:
    Level level_;
    std::string_view file_;
    int line_;
    bool enabled_;
    std::ostringstream stream_;
};

} // namespace ipn_mpc::logging

#define IPN_LOG_DEBUG                                                                              \
    ::ipn_mpc::logging::LogLine(::ipn_mpc::logging::Level::debug, __FILE__, __LINE__)
#define IPN_LOG_INFO                                                                               \
    ::ipn_mpc::logging::LogLine(::ipn_mpc::logging::Level::info, __FILE__, __LINE__)
#define IPN_LOG_WARNING                                                                            \
    ::ipn_mpc::logging::LogLine(::ipn_mpc::logging::Level::warning, __FILE__, __LINE__)
#define IPN_LOG_ERROR                                                                              \
    ::ipn_mpc::logging::LogLine(::ipn_mpc::logging::Level::error, __FILE__, __LINE__)
