
class Chassis {
public:
    Chassis(MotorBase *motor1, MotorBase *motor2, MotorBase *motor3, MotorBase *motor4, Hwt101Parser& imu) : motors_{motor1, motor2, motor3, motor4}, imu_(imu) {};
    ~Chassis() = default;

    std::array<float, 4> velocity_to_cmd(float vx, float vy, float omega) const {
        std::array<float, 4> cmds{};
        
        return cmds;
    }

    void set_velocity(float vx, float vy, float omega) {

    }

private:
    MotorBase *motors_[4];
    Hwt101Parser& imu_;



};

void newChassisTask(void *pvParameters);
