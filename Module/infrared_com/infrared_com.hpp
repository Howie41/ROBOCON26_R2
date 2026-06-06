/**
 * @file infrared_com.hpp
 * @author zhy (Howie41)
 * @brief 红外通信模块
 * @date 2026-05-16
 *
 * @note 模块使用了CMSIS-OS API，务必在RTOS内核后启动后再发送数据！
 * @note 模块波特率默认为9600bps
 */

#pragma once
#include "cmsis_os2.h"
#include "main.h"
#include "UartPort.hpp"
#include "stm32h7xx_hal_uart.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "usart.h"

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <array>
#include <optional>

class InfraredModule {
    public:
        typedef struct {
            uint16_t uid;
            uint8_t data;
        } infrared_msg_t;

        explicit InfraredModule(UartPort &uart_port) : uart_port_(uart_port) {}
        ~InfraredModule() = default;

        /**
         * @brief 由 UartPort 的接收回调转发进来
         * @param data 新收到的数据片段
         * @param len 数据长度
         * @note 帧格式：0xAA 0xXX uid_lo uid_hi 0xXX 0xBB
         */
        static constexpr uint8_t HEADER = 0xAA;
        static constexpr uint8_t FOOTER = 0xBB;
        static constexpr size_t RAW_LENGTH = 6; // 包括头尾的总长度
        static constexpr size_t MAX_SEARCH_LENGTH = 3; // 最大搜索长度
        void UartPortRxCbHandler(const uint8_t *data, size_t len) {
            if (data == nullptr || len == 0) return;
                if (len < MAX_SEARCH_LENGTH + RAW_LENGTH) return; // 不足以搜索一个完整包，丢弃
                for (size_t i = 0; i <= MAX_SEARCH_LENGTH; ++i) {
                    if (
                        data[i] == HEADER && data[i + RAW_LENGTH - 1] == FOOTER // 检查帧头帧尾
                        && data[i + 1] == data[i + 4] // 两个编码一致
                    ) {
                        infrared_msg_t ir_msg{};
                        ir_msg.uid = static_cast<uint16_t>(data[i + 3]) << 8 | data[i + 2];
                        ir_msg.data = data[i + 4];
                        latest_msg_ = ir_msg;
                        break; // 找到一个完整包后停止搜索
                    }
                }
        }

        infrared_msg_t getLatestMsg() { return latest_msg_; };

    private:
        UartPort &uart_port_;

        infrared_msg_t latest_msg_{};
};

class InfraredModuleGroup {
    public:
        static constexpr size_t MAX_MODULE_NUM = 4;

        InfraredModuleGroup() = default;
        ~InfraredModuleGroup() = default;
    
    private:
        uint16_t max_uid_received_ = 0;
        std::array<std::optional<InfraredModule>, MAX_MODULE_NUM> infrared_modules_;

        // TypedTopicPublisher<InfraredModule::infrared_msg_t> infrared_pub_{"infrared_msg"};
        // InfraredModule::infrared_msg_t infrared_msg_{};

    public:
        bool addModule(const InfraredModule &module) {
            for (auto &m : infrared_modules_) {
                if (!m.has_value()) {
                    m.emplace(module);
                    return true;
                }
            }
            return false;
        }

        std::optional<InfraredModule::infrared_msg_t> tryGet() {
            uint16_t temp_max_uid = max_uid_received_;
            std::optional<InfraredModule::infrared_msg_t> valid_latest_msg{std::nullopt};
            for (auto &m : infrared_modules_) {
                if (m.has_value()) {
                    InfraredModule::infrared_msg_t msg = m.value().getLatestMsg();
                    if (msg.uid > temp_max_uid) {
                        valid_latest_msg.emplace(msg);
                        temp_max_uid = msg.uid;
                    }
                }
            }
            max_uid_received_ = temp_max_uid;
            return valid_latest_msg;
        }

        uint16_t getMaxUidReceived() const { return max_uid_received_; }
};