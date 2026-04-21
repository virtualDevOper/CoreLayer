#include "Logger.h"
#include <filesystem>

void Logger::configure(const bool log_to_file, const bool log_to_console, const std::string& log_dir) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    log_to_file_ = log_to_file;
    log_to_console_ = log_to_console;
    
    if (log_to_file_) {
        // Создать директорию, если не существует
        std::filesystem::create_directories(log_dir);
        
        // Сгенерировать имя файла с timestamp
        const auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now{};
        
        #ifdef _WIN32
            localtime_s(&tm_now, &time_t_now);
        #else
            localtime_r(&time_t_now, &tm_now);
        #endif
        
        std::ostringstream filename;
        filename << log_dir;
        if (!log_dir.empty() && log_dir.back() != '/' && log_dir.back() != '\\') {
            filename << "/";
        }
        filename << "simulation_"
                 << std::put_time(&tm_now, "%d_%m_%Y__%H_%M_%S")
                 << ".log";
        
        log_file_path_ = filename.str();
        log_file_.open(log_file_path_, std::ios::out | std::ios::app);
        
        if (!log_file_.is_open()) {
            std::cerr << "Warning: Failed to open log file: " << log_file_path_ << std::endl;
            log_to_file_ = false;
        }
    }
}

Logger::~Logger() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void Logger::info(const std::string& message) {
    log(Level::INFO, message);
}

void Logger::warning(const std::string& message) {
    log(Level::WARNING, message);
}

void Logger::error(const std::string& message) {
    log(Level::ERROR, message);
}

void Logger::log(Level level, const std::string& message) {
    std::lock_guard<std::mutex> lock(log_mutex_);

    const std::string timestamp = getCurrentTimestamp();
    const std::string level_str = levelToString(level);
    const std::string formatted_message = "[" + timestamp + "] [" + level_str + "] " + message;
    
    if (log_to_console_) {
        if (level == Level::ERROR) {
            std::cerr << formatted_message << std::endl;
        } else {
            std::cout << formatted_message << std::endl;
        }
    }
    
    if (log_to_file_ && log_file_.is_open()) {
        log_file_ << formatted_message << std::endl;
        log_file_.flush(); // Немедленная запись для критичных логов
    }
}

std::string Logger::getCurrentTimestamp() {
    const auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now{};
    
    #ifdef _WIN32
        localtime_s(&tm_now, &time_t_now);
    #else
        localtime_r(&time_t_now, &tm_now);
    #endif
    
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::string Logger::levelToString(Level level) {
    switch (level) {
        case Level::INFO:    return "INFO";
        case Level::WARNING: return "WARNING";
        case Level::ERROR:   return "ERROR";
        default:             return "UNKNOWN";
    }
}
