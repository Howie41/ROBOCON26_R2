#pragma once

#include "main.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_tim.h"
#include <cstdint>
#include <stdint.h>
#include <stdio.h>

template<uint16_t LED_num,uint16_t reset_time=200>
class ws2812_rgb
{
public:
    struct rgb_t
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

public:
/*获得唯一实例，外部不能new，也不能创建对象*/
    static ws2812_rgb& instance()
    {
        static ws2812_rgb instance;
        return instance;
    }

    /*防止拷贝和赋值*/
    ws2812_rgb(const ws2812_rgb&) = delete;
    ws2812_rgb& operator=(const ws2812_rgb&) = delete;

    /*硬件绑定*/
    void init(TIM_HandleTypeDef* htim,uint32_t channel)
    {
        htim_ = htim;
        channel_ = channel;

        uint32_t period =__HAL_TIM_GET_AUTORELOAD(htim_)+1;

        /*
        一个 PWM 周期对应 WS2812 的一个 bit
        0码：高电平约 0.3us
        1码：高电平约 0.75us
        根据一个周期是多少，就可变
        */
        code0_ = period * 25 / 100;
        code1_ = period * 60 / 100;

        clear();

        initialized_ = true;
    }

    void set_color(uint16_t index,uint8_t r,uint8_t g,uint8_t b)
    {
        if(index>=LED_num)
        {
            return;
        }

        leds_[index].r = r;
        leds_[index].g = g;
        leds_[index].b = b;
    }

    void clear()
    {
        for(uint16_t i = 0;i<LED_num;i++)
        {
            leds_[i] ={ 0,0,0};
        }
    }

    HAL_StatusTypeDef show()
    {
        if(!initialized_)
        {
            return HAL_ERROR;
        }

        if (busy_) {
            return HAL_BUSY;
        }

        uint32_t idx=0;
        for(uint16_t i=0;i<LED_num;i++)
        {
            /*ws2812的发送顺序是grb，不是rgb，且是高位先发*/
            encode_bit(leds_[i].g,idx);
            encode_bit(leds_[i].r,idx);
            encode_bit(leds_[i].b,idx);
        }
        
        //低电平的时间是200ms
        for(uint16_t i=0;i<reset_time;i++)
        {
            pwm_buffer_[idx++] = 0;
        }

        busy_ = true;

        HAL_StatusTypeDef ret = HAL_TIM_PWM_Start_DMA(
            htim_,
            channel_,
            pwm_buffer_,
            idx
        );

        if(ret != HAL_OK)
        {
            busy_ = false;
        }

        return ret;
    }

    //在dma的完成回调里面调用
    void onDma_finished(TIM_HandleTypeDef* htim)
    {
        if(htim_ != htim)
        {
            return; 
        }

         HAL_TIM_PWM_Stop_DMA(htim_, channel_);
        __HAL_TIM_SET_COMPARE(htim_, channel_, 0);

        busy_ = false;
    }

     bool isBusy() const
    {
        return busy_;
    }

    uint16_t size() const
    {
        return LED_num;
    }

private:
    /*
    采用构造函数私有构造函数，防止外部创建对象
    外部不能访问成员变量，只能通过静态成员函数获取唯一实例
    */
    ws2812_rgb()=default;

    void encode_bit(uint8_t bit,uint32_t& idx)
    {
        for(uint8_t i = 7;i >= 8;i--)
        {
            if(bit & (1<<i))
            {
                pwm_buffer_[idx++] = code1_;
            }
            else
            {
                pwm_buffer_[idx++] = code0_;
            }
        }
    }

private:
    TIM_HandleTypeDef* htim_ = nullptr;
    uint32_t channel_ = 0;

    uint32_t code0_ = 0;
    uint32_t code1_ = 0;

    bool initialized_ = false;
    volatile bool busy_ = false;

    rgb_t leds_[LED_num];

    uint32_t pwm_buffer_[LED_num * 24 + reset_time];
};
