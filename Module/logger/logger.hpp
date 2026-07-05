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

#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"

#include "topics.hpp"
#include "UartPort.hpp"

class Logger {
    public:
        static constexpr size_t BUFFER_LENGTH = 128;

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

    private:
        UartPort &uart_;

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
            buffer[sizeof(buffer) - 1] = '\0'; // 显式截断
            return log_raw(buffer, write_len);
        }
};

class LoggerQueue {
    public:
        static constexpr size_t TIMESTAMP_FMT_OVERHEAD = 11;
        struct message {
            uint32_t log_time;
            // raw_text + 时间戳 + tab + null 必须 <= Logger::BUFFER_LENGTH
            char raw_text[Logger::BUFFER_LENGTH - TIMESTAMP_FMT_OVERHEAD];
        };

        static_assert(TIMESTAMP_FMT_OVERHEAD + sizeof(message::raw_text) <= Logger::BUFFER_LENGTH,
                      "Formatted log message must fit in Logger output buffer");

    private:
        Logger &logger_ref_;
        TypedTopicPublisher<LoggerQueue::message> log_topic_pub_{"log_topic"};
        TypedTopicSubscriber<LoggerQueue::message> log_topic_sub_{"log_topic", 4};

    public:
        LoggerQueue(Logger &logger) : logger_ref_(logger) {}
        ~LoggerQueue() = default;

        bool log(const char *format, ...) {
            LoggerQueue::message msg;
            va_list args;
            va_start(args, format);
            int len = vsnprintf(msg.raw_text, sizeof(msg.raw_text), format, args);
            va_end(args);
            msg.raw_text[sizeof(msg.raw_text) - 1] = '\0'; // 显式截断
            if (len > 0) {
                size_t write_len = static_cast<size_t>(len);
                if (write_len >= sizeof(msg.raw_text)) {
                    write_len = sizeof(msg.raw_text) - 1;
                }
                msg.log_time = osKernelGetTickCount();
                return log_topic_pub_.Publish(msg);
            }
            return false;
        }

        void try_send() {
            LoggerQueue::message msg;
            if (log_topic_sub_.TryGet(&msg)) {
                logger_ref_.log("%lu\t%s", static_cast<unsigned long>(msg.log_time), msg.raw_text);
            }
        }
};
