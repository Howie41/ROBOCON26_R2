/** 
 * @file filters.hpp
 * @author FunFer
 * @brief 滤波器类
 * @version 0.1
 * @date 2026-06-23
 */
#pragma once
#include <cmath>
#include <algorithm>


// 一阶低通滤波器
class LowPassFilter {
public:
    LowPassFilter() : alpha(0.5f), y_prev(0.0f), initialized(false) {}
    
    LowPassFilter(float cutoff_freq, float sample_freq) {
        init(cutoff_freq, sample_freq);
    }
    
    void init(float cutoff_freq, float sample_freq) {
        if (cutoff_freq <= 0 || sample_freq <= 0) {
            alpha = 0.5f;
            return;
        }
        float dt = 1.0f / sample_freq;
        float RC = 1.0f / (2.0f * M_PI * cutoff_freq);
        alpha = dt / (RC + dt);
        reset();
    }
    
    float update(float x) {
        if (!initialized) {
            y_prev = x;
            initialized = true;
        }
        y_prev = alpha * x + (1.0f - alpha) * y_prev;
        return y_prev;
    }
    
    void reset() { 
        initialized = false; 
        y_prev = 0.0f;
    }
    
    inline float getAlpha() const { return alpha; }
    inline void setAlpha(float a) { alpha = a; }
    
private:
    float alpha;
    float y_prev;
    bool initialized;
};

// 移动平均滤波器
template<size_t WINDOW_SIZE>
class MovingAverageFilter {
public:
    MovingAverageFilter() : index(0), count(0), sum(0.0f) {
        for (size_t i = 0; i < WINDOW_SIZE; i++) buffer[i] = 0.0f;
    }
    
    float update(float x) {
        if (count == WINDOW_SIZE) sum -= buffer[index];
        else count++;
        buffer[index] = x;
        sum += x;
        index++;
        if (index >= WINDOW_SIZE) index = 0;
        return sum / (float)count;
    }
    
    void reset() {
        for (size_t i = 0; i < WINDOW_SIZE; i++) {
            buffer[i] = 0.0f;
        }
        index = 0;
        count = 0;
        sum = 0.0f;
    }
    
    inline size_t size() const { return count; }
    inline float getSum() const { return sum; }
    
private:
    float buffer[WINDOW_SIZE];
    size_t index;
    size_t count;
    float sum;
};

// 一维卡尔曼滤波器
class KalmanFilter1D {
public:
    /**
     * @param process_noise 过程噪声（模型不确定性），越小滤波越平滑
     * @param measure_noise 测量噪声（传感器噪声），越小越相信测量值
     */
    KalmanFilter1D(float process_noise = 0.001f, float measure_noise = 0.1f) : x(0.0f), P(1.0f), Q(process_noise), R(measure_noise), initialized(false) {}

    float update(float z) {
        if (!initialized) {
            x = z;
            P = 1.0f;
            initialized = true;
            return x;
        }
        float P_pred = P + Q;
        float K = P_pred / (P_pred + R);
        x = x + K * (z - x);
        P = (1.0f - K) * P_pred;
        return x;
    }
    void reset() {
        initialized = false;
        x = 0.0f;
        P = 1.0f;
    }
    inline void setQ(float q) { Q = q; }
    inline void setR(float r) { R = r; }
    inline float getEstimate() const { return x; }
    
private:
    float x;
    float P;
    float Q;
    float R;
    bool initialized;
};
