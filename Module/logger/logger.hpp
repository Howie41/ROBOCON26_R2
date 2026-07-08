/**
 * @file logger.hpp
 * @author zhy (Howie41)
 * @brief 从串口打印简单信息
 * @date 2026-05-23
 */
#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"

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
    static constexpr size_t FMT_PREFIX_MAX = 11;  // "%lu\t" 最大宽度: 10(timestamp) + 1(tab)
    struct message {
        uint32_t log_time;
        // raw_text + timestamp + tab + null 必须 <= Logger::BUFFER_LENGTH
        char raw_text[Logger::BUFFER_LENGTH - FMT_PREFIX_MAX];

        /// 格式化为 "timestamp\traw_text"，返回实际字节数（不含 null）
        size_t format_to(char *buf, size_t buf_size) const {
            int len = snprintf(buf, buf_size, "%lu\t%s",
                               static_cast<unsigned long>(log_time), raw_text);
            if (len <= 0) return 0;
            size_t n = static_cast<size_t>(len);
            return (n < buf_size) ? n : buf_size - 1;
        }
    };

    static_assert(FMT_PREFIX_MAX + sizeof(message::raw_text) <= Logger::BUFFER_LENGTH,
                    "Formatted log message must fit in Logger output buffer");

private:
    Logger &logger_ref_;
    osMessageQueueId_t log_queue_handle_{nullptr};   // → debugTask → UART10
    osMessageQueueId_t pc_queue_handle_{nullptr};    // → PcComTask → USB 上位机

public:
    LoggerQueue(Logger &logger) : logger_ref_(logger) {}
    ~LoggerQueue() = default;

    /// 必须在 osTaskInit() 中（队列创建后、任务启动前）调用
    void init(osMessageQueueId_t log_handle, osMessageQueueId_t pc_handle) {
        log_queue_handle_ = log_handle;
        pc_queue_handle_ = pc_handle;
    }

    bool log(const char *format, ...) {
        LoggerQueue::message msg;
        va_list args;
        va_start(args, format);
        int len = vsnprintf(msg.raw_text, sizeof(msg.raw_text), format, args);
        va_end(args);
        msg.raw_text[sizeof(msg.raw_text) - 1] = '\0'; // 显式截断
        if (len > 0) {
            msg.log_time = osKernelGetTickCount();
            // 非阻塞推送到两个队列；队列满时丢弃
            bool ok = true;
            if (log_queue_handle_ != nullptr) {
                ok &= (osMessageQueuePut(log_queue_handle_, &msg, 0U, 0U) == osOK);
            }
            if (pc_queue_handle_ != nullptr) {
                osMessageQueuePut(pc_queue_handle_, &msg, 0U, 0U);
            }
            return ok;
        }
        return false;
    }

    void try_send() {
        if (log_queue_handle_ == nullptr) {
            return;
        }
        LoggerQueue::message msg;
        if (osMessageQueueGet(log_queue_handle_, &msg, NULL, 0U) == osOK) {
            char formatted[Logger::BUFFER_LENGTH];
            size_t n = msg.format_to(formatted, sizeof(formatted));
            if (n > 0) {
                logger_ref_.log_raw(formatted, n);
            }
        }
    }
};
