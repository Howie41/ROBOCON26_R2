/**
 * @file logger.hpp
 * @author zhy (Howie41)
 * @brief 从串口打印简单信息
 * @date 2026-05-23
 */
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "stm32h7xx_hal.h"

#include "topics.hpp"
#include "UartPort.hpp"

class Logger {
    public:
        static constexpr size_t BUFFER_LENGTH = 256;

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

        HAL_StatusTypeDef log_priority(uint8_t priority, const char *format, ...) {
            if (priority < current_priority_) {
                return HAL_ERROR;
            }

            va_list args;
            va_start(args, format);
            auto result = format_raw(format, args);
            va_end(args);
            return result;
        }

        void set_priority(uint8_t priority) {
            current_priority_ = priority;
        }

    private:
        UartPort &uart_;
        uint8_t current_priority_{0}; // 默认允许所有优先级输出

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
