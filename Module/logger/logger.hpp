/**
 * @file logger.hpp
 * @author zhy (Howie41)
 * @brief 从串口打印简单信息
 * @date 2026-05-23
 */
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "stm32h7xx_hal.h"

#include "topics.hpp"
#include "UartPort.hpp"

class Logger {
    public:
        static constexpr size_t BUFFER_LENGTH = 128;
        enum class LogLevel : uint8_t {
            debug = 0,
            info = 1,
            warn = 2,
            error = 3,
        };

        Logger(UartPort &uart) : uart_(uart) {}

        HAL_StatusTypeDef log_raw(const char *data, size_t len) {
            return uart_.write(reinterpret_cast<const uint8_t *>(data), len);
        }

        HAL_StatusTypeDef log(const char *format, ...) {
            va_list args;
            va_start(args, format);
            auto result = format_raw(format, args);
            va_end(args);
            return result;
        }

        HAL_StatusTypeDef log_level(LogLevel level, const char *format, ...) {
            if (static_cast<uint8_t>(level) < static_cast<uint8_t>(current_level_)) {
                return HAL_ERROR;
            }

            va_list args;
            va_start(args, format);
            auto result = format_raw(format, args);
            va_end(args);
            return result;
        }

        void set_level(LogLevel level) {
            current_level_ = level;
        }

    private:
        UartPort &uart_;
        LogLevel current_level_{LogLevel::debug};

        HAL_StatusTypeDef format_raw(const char *format, va_list args) {
            char buffer[BUFFER_LENGTH];
            int len = vsnprintf(buffer, sizeof(buffer), format, args);
            if (len <= 0) {
                return HAL_ERROR;
            }

            size_t write_len = static_cast<size_t>(len);
            if (write_len >= sizeof(buffer)) {
                write_len = sizeof(buffer) - 1;
            }
            return log_raw(buffer, write_len);
        }
};

class LoggerQueue {
    private:
        Logger &logger_ref_;
        TypedTopicPublisher<char[Logger::BUFFER_LENGTH]> log_topic_pub_{"log_topic"};
        TypedTopicSubscriber<char[Logger::BUFFER_LENGTH]> log_topic_sub_{"log_topic", 10};

    public:
        LoggerQueue(Logger &logger) : logger_ref_(logger) {}
        ~LoggerQueue() = default;

        bool log(const char *format, ...) {
            char buffer[Logger::BUFFER_LENGTH];
            va_list args;
            va_start(args, format);
            int len = vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            if (len > 0) {
                size_t write_len = static_cast<size_t>(len);
                if (write_len >= sizeof(buffer)) {
                    write_len = sizeof(buffer) - 1;
                }
                return log_topic_pub_.Publish(buffer);
            }
            return false;
        }

        void trySend() {
            char buffer[Logger::BUFFER_LENGTH];
            if (log_topic_sub_.TryGet(&buffer)) {
                logger_ref_.log_raw(buffer, std::strlen(buffer));
            }
        }
};
