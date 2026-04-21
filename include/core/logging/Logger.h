#pragma once
#include "../PCH.h"

/**
 * @brief Система логирования с поддержкой записи в файл и консоль
 * 
 * Singleton-класс для централизованного логирования событий симуляции.
 * Поддерживает три уровня: INFO, WARNING, ERROR.
 */
class Logger {
public:
    enum class Level {
        INFO,
        WARNING,
        ERROR
    };
    
    /**
     * @brief Получить единственный экземпляр логгера
     */
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }
    
    /**
     * @brief Настроить логгер
     * @param log_to_file Записывать ли логи в файл
     * @param log_to_console Выводить ли логи в консоль
     * @param log_dir Директория для файлов логов
     */
    void configure(bool log_to_file, bool log_to_console, const std::string& log_dir);
    
    /**
     * @brief Логировать информационное сообщение
     */
    void info(const std::string& message);
    
    /**
     * @brief Логировать предупреждение
     */
    void warning(const std::string& message);
    
    /**
     * @brief Логировать ошибку
     */
    void error(const std::string& message);
    
    // Запрет копирования и перемещения
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;
    
private:
    Logger() = default;
    ~Logger();
    
    /**
     * @brief Внутренний метод логирования
     */
    void log(Level level, const std::string& message);
    
    /**
     * @brief Получить текущий timestamp в формате [YYYY-MM-DD HH:MM:SS]
     */
    static std::string getCurrentTimestamp() ;
    
    /**
     * @brief Преобразовать уровень в строку
     */
    static std::string levelToString(Level level) ;
    
    bool log_to_file_ = true;
    bool log_to_console_ = true;
    std::string log_file_path_;
    std::ofstream log_file_;
    std::mutex log_mutex_;
};
