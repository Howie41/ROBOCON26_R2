#ifndef TAIL_CLAW_CONTROLLER_HPP
#define TAIL_CLAW_CONTROLLER_HPP

#include "Motor.hpp"
#include "FreeRTOS.h"
#include "pid_controller.h"
#include "task.h"
#include "topics.hpp"
#include "topic_pool.h"
#include <cstdint>

//状态枚举
typedef enum weapon_match_state {
    none = 0x00,
    motor_move_left = 0x01,
    motor_move_right = 0x02,
    motor_roll_up = 0x04,
    motor_roll_down = 0x08,
    ismatch = 0x10,
} weapon_match_state;


class TailClawController {
public:
    //获取尾爪控制器单例对象。
    static TailClawController& Instance();

    //初始化尾爪控制器，绑定横移电机和翻转电机，并初始化 PID。
    void Init(C610Motor& move_motor, C620Motor& roll_motor);

    //1ms 周期调度入口，处理命令、输入、目标更新、GPIO 和电机输出。
    void Tick1ms();

    //处理状态机或其他任务发来的尾爪控制命令
    void HandleCommand(const pub_tail_claw_cmd& cmd);

    //接收上位机发送的尾爪对齐距离数据
    void OnPcDistance(int16_t distance);

    //设置尾爪工作模式。
    void SetMode(TailClawMode mode);

    /**
     * 设置尾爪横移目标位置。cm 横移目标位置，单位 cm。
     */
    void SetMoveTarget(float cm);

    /**
     * 设置尾爪翻转目标角度。deg 翻转目标角度，单位 deg。
     */
    void SetRollTarget(float deg);

    /**
     *  false 表示夹紧，ture 表示松开。
     */
    void SetWeaponClaw(bool close);

    /**
     * 设置气泵输出状态。 true 表示打开气泵，false 表示关闭气泵。
     */
    void SetAirPump(bool on);

    /*
      清除自动对齐过程中的匹配计数和匹配状态。
     */
    void ResetMatchState();

    /**
     * @brief 获取当前尾爪工作模式。
     */
    TailClawMode GetMode() const { return mode_; }

    /*
    获取当前尾爪状态快照。 当前模式、目标位置、夹爪状态和到位状态。
     */
    pub_tail_claw_status GetStatus() const;

private:
    /*
     * 私有构造函数，保证外部只能通过 Instance() 获取对象。
     */
    TailClawController() = default;

     //禁止复制构造，避免出现多个尾爪控制器实例。
    TailClawController(const TailClawController&) = delete;

    // 禁止赋值，避免覆盖单例内部状态。
     TailClawController& operator=(const TailClawController&) = delete;

    //初始化横移和翻转电机使用的四个 PID 参数。
    void init_pid();

    //消费 tail_claw_cmd topic 中的所有待处理命令。
    void consume_commands();

    //消费上位机发布的尾爪距离数据。
    void consume_pc_distance();

    //消费 xbox topic 中的最新手柄输入。
    void consume_xbox_data();

    //根据当前模式、上位机距离和手柄输入生成 motion_bits_。
    void update_control_bits();

    //根据上位机距离数据更新自动对齐方向和匹配稳定状态。
    void update_auto_align(const tail_claw_msg* msg);

    // 根据手柄方向键更新横移和翻转控制方向。
    void update_manual_control();

    //根据手柄按键上升沿切换夹爪和气泵状态。
    void update_button_toggle();

    //根据 motion_bits_ 推进横移目标位置和翻转目标角度。
    void update_target_by_control_bits();

    //将夹爪和气泵状态输出到 GPIO。
    void apply_gpio();

    //计算并设置横移电机和翻转电机电流命令
    void apply_motor_output();

    //周期性发布尾爪状态到 tail_claw_status topic。
    void publish_status();

    //判断上位机距离数据是否仍在有效时间内。true 表示距离数据新鲜，false 表示没有数据或数据超时
    bool is_distance_fresh() const;

    //将当前电机位置记录为 Hold 模式下的目标位置。
    void lock_current_position();

    //根据电机当前位置和速度更新状态结构体中的到位标志。要补充到位信息的状态结构体指针。
    void update_arrived_status(pub_tail_claw_status* status) const;

    /**
     * @brief 计算尾爪横移电机输出命令。
     * @param target_cm 横移目标位置，单位 cm。
     * @return 横移电机输出命令。
     */
    float calcMoveCmd(float target_cm);

    /**
     * @brief 计算尾爪翻转电机输出命令。
     * @param target_deg 翻转目标角度，单位 deg。
     * @return 翻转电机输出命令。
     */
    float calcRollCmd(float target_deg);

public:
    C610Motor* move_motor_{nullptr};
    C620Motor* roll_motor_{nullptr};

    PID_t move_pos_pid_{};
    PID_t move_speed_pid_{};
    PID_t roll_pos_pid_{};
    PID_t roll_speed_pid_{};

    TailClawMode mode_{TailClawMode::AutoAlign};
    uint8_t motion_bits_{0};

    bool weapon_claw_open_{false};
    bool air_pump_on_{false};

    bool btn_share_last_{false};
    bool btn_menu_last_{false};

    float move_target_cm_{7.0f};
    float roll_target_deg_{0.0f};

    int16_t last_distance_{0};
    bool has_last_distance_{false};
    TickType_t last_distance_tick_{0};

    uint8_t match_ok_count_{0};
    uint8_t match_lost_count_{0};
    bool weapon_matched_stable_{false};

    uint8_t status_publish_count_{0};

    pub_Xbox_Data xbox_cmd_{};

    // 状态机或其他任务发给尾爪控制器的高层控制命令。
    TypedTopicSubscriber<pub_tail_claw_cmd> tail_claw_cmd_sub_{"tail_claw_cmd", 4};

    // 上位机发送的尾爪对齐距离数据，用于 AutoAlign 模式。
    TypedTopicSubscriber<tail_claw_msg> pc_tail_claw_sub_{"pc_tail_claw_pub", 8};

    // 手柄输入数据，用于手动控制和按键上升沿切换。
    TypedTopicSubscriber<pub_Xbox_Data> xbox_sub_{"xbox", 8};

    // 尾爪控制器对外发布的状态快照，供状态机等待到位或对齐。
    TypedTopicPublisher<pub_tail_claw_status> tail_claw_status_pub_{"tail_claw_status"};
};

#endif
