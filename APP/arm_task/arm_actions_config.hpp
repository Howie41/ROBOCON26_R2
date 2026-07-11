/**
 * @file arm_task.cpp
 * @author FunFer
 * @brief 取矿原子动作姿态点位数据
 * @copyright Copyright (c) 2026
 */

#include "memory_map.h"

// 为了避免不必要的麻烦，直接用cpp当json写了

enum special_operations_enum {
    SKIP_MOTOR_CONTROL_ = 0b00000001,
    RESET_ = 0b00000010,
    FETCH_ = 0b00000100,
    RELEASE_ = 0b00001000
};

struct height {float pos; float speed; };
struct flip { float pos; float speed; };
struct rotate { float pos; float speed; };
struct expand { float pos; float speed; };
struct arm_pose {  // 三个构造函数：第一个是正常构造姿态，第二个是特殊操作（不构造姿态），第三个是结束标志
    float delta_t{0.0f}; height h; flip f; rotate r; expand e; uint8_t special_operations; bool is_end{false};
    arm_pose(struct height h, struct flip f, struct rotate r, struct expand e, uint8_t so = 0) : h(h), f(f), r(r), e(e), special_operations(so) {}
    arm_pose(float delta_t, struct height h, struct flip f, struct rotate r, struct expand e, uint8_t so = 0) : delta_t(delta_t), h(h), f(f), r(r), e(e), special_operations(so) {}
    arm_pose(float delta_t, uint8_t so) : delta_t(delta_t), h({0.0f, 0.0f}), f({0.0f, 0.0f}), r({0.0f, 0.0f}), e({0.0f, 0.0f}), special_operations(so | SKIP_MOTOR_CONTROL_) {}
    arm_pose(float delta_t) : delta_t(delta_t), special_operations(SKIP_MOTOR_CONTROL_), is_end(true) {}
};

/** arm_pose前四个成员分别代表：height, flip, rotate, expand
 * special_operations各位代表特殊操作，详见枚举special_operations_enum
 */

namespace arm_actions_config {
    RAM_D1_ATTR const arm_pose start_proceed[] = {  // 启动姿态
        { 0.01f, {570.0f, 1000.0f}, {90.0f, 120.0f}, {-85.0f, 2.3f}, {630.0f, 20.0f} },
        { 0.6f, {600.0f, 1000.0f}, {0.0f, 120.0f}, {0.0f, 2.3f}, {560.0f, 20.0f} },
        { 0.5f, RESET_ },
        { 0.01f }
    };
    // 复位姿态
    RAM_D1_ATTR const arm_pose reset = { {570.0f, 1000.0f}, {0.0f, 120.0f}, {0.0f, 3.0f}, {0.0f, 18.0f} };

    namespace fetch_proceed {
        // 吸取 +200 台阶 KFS
        RAM_D1_ATTR const arm_pose step_M[] = {  // height, flip, rotate, expand
            { 0.01f, {520.0f, 1000.0f}, {82.0f, 120.0f}, {9.0f, 2.2f}, {1080.0f, 20.0f} },
            { 0.78f, {365.0f, 1000.0f}, {82.0f, 120.0f}, {21.5f, 2.1f}, {1080.0f, 20.0f}, FETCH_ },
            { 0.47f, {660.0f, 1000.0f}, {82.0f, 120.0f}, {0.0f, 2.1f}, {600.0f, 18.0f} },
            { 0.3, {570.0f, 1000.0f}, {-90.0f, 100.0f}, {0.0f, 2.8f}, {0.0f, 20.0f} },
            { 0.01f }
        };
        // 吸取 +400 台阶 KFS（存了0个）
        RAM_D1_ATTR const arm_pose step_H[] = {
            { 0.01f, {600.0f, 1000.0f}, {90.0f, 120.0f}, {-11.0f, 2.2f}, {1170.0f, 21.0f} },
            { 1.25f, {590.0f, 1000.0f}, {90.0f, 120.0f}, {3.0f, 2.1f}, {1170.0f, 18.0f}, FETCH_ },
            { 0.4f, {920.0f, 1000.0f}, {88.0f, 120.0f}, {-9.0f, 2.0f}, {800.0f, 18.0f} },
            { 0.3f, {570.0f, 1000.0f}, {-90.0f, 90.0f}, {0.0f, 2.8f}, {0.0f, 21.0f} },  // 加速处理
            { 0.01f }
        };
        // 吸取 -200 台阶 KFS（存了0个）
        RAM_D1_ATTR const arm_pose step_L[] = {
            { 0.01f, {-240.0f, 1000.0f}, {56.5f, 120.0f}, {35.0f, 2.1f}, {1080.0f, 18.0f}, FETCH_ },
            { 2.35f, {660.0f, 1000.0f}, {80.0f, 120.0f}, {35.0f, 2.1f}, {700.0f, 18.0f} },
            { 0.28f, {570.0f, 1000.0f}, {-5.0f, 80.0f}, {0.0f, 3.0f}, {680.0f, 17.0f} },
            { 1.2f, {570.0f, 1000.0f}, {-90.0f, 70.0f}, {0.0f, 3.0f}, {0.0f, 17.0f} },
            { 0.01f }
        };
        // 吸取 +0 台阶 KFS（存了0个）
        RAM_D1_ATTR const arm_pose step_P[] = {
            { 0.01f, {500.0f, 1000.0f}, {50.0f, 120.0f}, {22.0f, 2.2f}, {1080.0f, 20.0f} },
            { 0.59f, {300.0f, 1000.0f}, {50.0f, 120.0f}, {44.0f, 2.1f}, {1080.0f, 18.0f}, FETCH_ },
            { 0.7f, {820.0f, 1000.0f}, {60.0f, 120.0f}, {25.0f, 2.0f}, {800.0f, 18.0f} },
            { 0.65f, {570.0f, 1000.0f}, {-90.0f, 110.0f}, {0.0f, 3.0f}, {0.0f, 20.0f} },
            { 0.01f }
        };
    }
    namespace place_proceed {
        // 取出KFS（存了1个）
        RAM_D1_ATTR const arm_pose kfs_1[] = {
            { 0.01f, {80.0f, 1000.0f}, {-78.0f, 120.0f}, {-192.0f, 6.8f}, {790.0f, 21.0f}, FETCH_ },
            { 2.55f, {1040.0f, 1000.0f}, {-106.0f, 90.0f}, {-160.0f, 2.7f}, {710.0f, 21.0f} },
            { 1.2f, {1020.0f, 1000.0f}, {-81.5f, 100.0f}, {-18.5f, 3.1f}, {660.0f, 18.0f} },
            { 1.25f, {1020.0f, 1000.0f}, {8.5f, 90.0f}, {-21.5f, 3.1f}, {660.0f, 18.0f} },
            { 0.01f }
        };
        // 取出KFS（存了2个）
        RAM_D1_ATTR const arm_pose kfs_2[] = {
            { 0.01f, {1040.0f, 1000.0f}, {-90.0f, 120.0f}, {0.0f, 3.1f}, {880.0f, 28.0f} },
            { 0.7f, {900.0f, 1000.0f}, {-88.0f, 120.0f}, {-200.0f, 6.8f}, {720.0f, 28.0f}, FETCH_ },
            { 2.2f, {1020.0f, 1000.0f}, {-81.5f, 100.0f}, {-18.5f, 3.1f}, {660.0f, 18.0f} },
            { 1.4f, {1020.0f, 1000.0f}, {8.5f, 90.0f}, {-21.5f, 3.1f}, {660.0f, 18.0f} },
            { 0.01f }
        };
        // 取出KFS（存了3个）
        RAM_D1_ATTR const arm_pose kfs_3[] = {
            { 0.01f, {1020.0f, 1000.0f}, {8.5f, 120.0f}, {-23.0f, 3.1f}, {660.0f, 18.0f} },
            { 0.01f }
        };
    }
    namespace place3_proceed {
        // 取出KFS（存了1个）（放置第三层）
        RAM_D1_ATTR const arm_pose kfs_1[] = {
            { 0.01f, {80.0f, 1000.0f}, {-78.0f, 120.0f}, {-192.0f, 6.8f}, {790.0f, 21.0f}, FETCH_ },
            { 2.55f, {1040.0f, 1000.0f}, {-106.0f, 90.0f}, {-160.0f, 2.7f}, {710.0f, 21.0f} },
            { 1.2f, {1020.0f, 1000.0f}, {-81.5f, 100.0f}, {-18.5f, 3.1f}, {660.0f, 18.0f} },
            { 1.1f, {1020.0f, 1000.0f}, {18.5f, 110.0f}, {-31.5f, 3.1f}, {660.0f, 18.0f} },
            { 0.01f }
        };
        // 取出KFS（存了2个）（放置第三层）
        RAM_D1_ATTR const arm_pose kfs_2[] = {
            { 0.01f, {1040.0f, 1000.0f}, {-90.0f, 120.0f}, {0.0f, 3.1f}, {880.0f, 28.0f} },
            { 0.7f, {900.0f, 1000.0f}, {-88.0f, 120.0f}, {-200.0f, 6.8f}, {720.0f, 28.0f}, FETCH_ },
            { 2.2f, {1020.0f, 1000.0f}, {-81.5f, 100.0f}, {-18.5f, 3.1f}, {660.0f, 18.0f} },
            { 1.4f, {1020.0f, 1000.0f}, {18.5f, 110.0f}, {-31.5f, 3.1f}, {660.0f, 18.0f} },
            { 0.01f }
        };
        // 取出KFS（存了3个）（放置第三层）
        RAM_D1_ATTR const arm_pose kfs_3[] = {
            { 0.01f, {1020.0f, 1000.0f}, {18.5f, 110.0f}, {-33.0f, 3.1f}, {660.0f, 18.0f} },
            { 0.01f }
        };
    }
    // 释放KFS并回归默认姿态
    RAM_D1_ATTR const arm_pose place_release_proceed[] = {
        { 0.01f, RELEASE_ },
        { 0.3f, {570.0f, 1000.0f}, {0.0f, 120.0f}, {12.0f, 2.1f}, {880.0f, 18.0f} },
        { 0.5f, RESET_ },
        { 0.01f }
    };
    namespace load_kfs_proceed {
        // 将 KFS 存入（当前剩0个KFS）
        RAM_D1_ATTR const arm_pose kfs_0[] = {
            { 0.01f, {680.0f, 1000.0f}, {-90.0f, 120.0f}, {-180.0f, 6.8f}, {460.0f, 18.0f} },
            { 0.8f, {500.0f, 1000.0f}, {-90.0f, 120.0f}, {-180.0f, 2.1f}, {420.0f, 18.0f} },
            { 0.7f, RELEASE_ },
            { 0.2f, {570.0f, 1000.0f}, {0.0f, 120.0f}, {0.0f, 2.2f}, {500.0f, 18.0f} },
            { 1.5f, RESET_ },
            { 0.01f }
        };
        // 将 KFS 存入（当前剩1个KFS）
        RAM_D1_ATTR const arm_pose kfs_1[] = {
            { 0.01f, {880.0f, 1000.0f}, {-92.0f, 120.0f}, {-180.0f, 6.8f}, {429.5f, 18.0f} },
            { 1.65f, RELEASE_ },
            { 0.2f, {670.0f, 1000.0f}, {0.0f, 120.0f}, {10.0f, 2.2f}, {780.0f, 18.0f} },
            { 1.5f, RESET_ },
            { 0.01f }
        };
    }
    namespace drop_kfs_proceed {
        // 将 KFS 丢掉（当前剩1或2个KFS）
        RAM_D1_ATTR const arm_pose kfs_12[] = {
            // { 0.01f, {680.0f, 1000.0f}, {-90.0f, 120.0f}, {-180.0f, 6.8f}, {460.0f, 18.0f} },
            // { 0.8f, {500.0f, 1000.0f}, {-90.0f, 120.0f}, {-180.0f, 2.1f}, {420.0f, 18.0f} },
            // { 0.7f, RELEASE_ },
            // { 0.2f, {570.0f, 1000.0f}, {0.0f, 120.0f}, {0.0f, 2.2f}, {500.0f, 18.0f} },
            // { 1.5f, RESET_ },
            { 0.01f }
        };
        // 将 KFS 丢掉（当前剩3个KFS）
        RAM_D1_ATTR const arm_pose kfs_3[] = {
            // { 0.01f, {880.0f, 1000.0f}, {-92.0f, 120.0f}, {-180.0f, 6.8f}, {429.5f, 18.0f} },
            // { 1.65f, RELEASE_ },
            // { 0.2f, {670.0f, 1000.0f}, {0.0f, 120.0f}, {10.0f, 2.2f}, {780.0f, 18.0f} },
            // { 1.5f, RESET_ },
            { 0.01f }
        };
    }
}
