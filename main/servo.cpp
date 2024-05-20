#include "servo.hpp"
#include "esp_log.h"

#define TAG "Servo"

// SG90サーボモーターのパルス幅設定
#define MIN_PULSEWIDTH 500  // 最小パルス幅 (単位: マイクロ秒)  この値はサーボ個体によって微調整が必要 500前後
#define MAX_PULSEWIDTH 2290 // 最大パルス幅 (単位: マイクロ秒)  この値はサーボ個体によって微調整が必要 2400前後
#define MIN_DEGREE -90      // 最大角度
#define MAX_DEGREE 90       // 最大角度

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000    // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000             // 20000 tick, 20ms


// メッセージ種別 (メッセージキュー用)
enum class ServoMessage {
    Angle       // 角度設定
};

static inline uint32_t example_angle_to_compare(double angle) {
    return (angle - MIN_DEGREE) * (MAX_PULSEWIDTH - MIN_PULSEWIDTH) / (MAX_DEGREE - MIN_DEGREE) + MIN_PULSEWIDTH;
}

Servo::Servo() {
    clear();
}

void Servo::clear() {
    m_xHandle = NULL;
    m_xQueue = NULL;
    m_gpio = 0;
    m_timer = NULL;
    m_oper = NULL;
    m_comparator = NULL;
    m_generator = NULL;
    m_angle = 0;
}

void Servo::init(uint8_t gpio, double angle) {
    m_gpio = gpio;
    m_angle = angle;

    // タスク作成
    xTaskCreate(Servo::task, TAG, configMINIMAL_STACK_SIZE * 2, (void*)this, 1, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(ServoMessage));

    // GPIO
    m_timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
    };
    esp_err_t ret = mcpwm_new_timer(&timer_config, &m_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_timer error %d", ret);
        return;
    }

    m_oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ret = mcpwm_new_operator(&operator_config, &m_oper);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_operator error %d", ret);
        return;
    }
    ret = mcpwm_operator_connect_timer(m_oper, m_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_operator_connect_timer error %d", ret);
        return;
    }

    m_comparator = NULL;
    mcpwm_comparator_config_t comparator_config {
        .flags = {
            .update_cmp_on_tez = true
        }
    };
    ret = mcpwm_new_comparator(m_oper, &comparator_config, &m_comparator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_comparator error %d", ret);
        return;
    }
    
    m_generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = m_gpio,
    };
    ret = mcpwm_new_generator(m_oper, &generator_config, &m_generator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_generator error %d", ret);
        return;
    }

    ret = mcpwm_comparator_set_compare_value(m_comparator, example_angle_to_compare(m_angle));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_comparator_set_compare_value error %d", ret);
        return;
    }

    ret = mcpwm_generator_set_actions_on_timer_event(m_generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
            // ,MCPWM_GEN_TIMER_EVENT_ACTION_END());
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_generator_set_actions_on_timer_event error %d", ret);
        return;
    }
    ret = mcpwm_generator_set_actions_on_compare_event(m_generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_comparator, MCPWM_GEN_ACTION_LOW));
            // ,MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_generator_set_actions_on_compare_event error %d", ret);
        return;
    }
    
    ret = mcpwm_timer_enable(m_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_enable error %d", ret);
        return;
    }
    ret = mcpwm_timer_start_stop(m_timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_start_stop error %d", ret);
        return;
    }

    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, m_gpio);
    // mcpwm_config_t pwm_config;
    // pwm_config.frequency = 50;  // 50HzのPWM信号
    // pwm_config.cmpr_a = 0;      // 初期デューティサイクル 0%
    // pwm_config.cmpr_b = 0;      // 初期デューティサイクル 0%
    // pwm_config.counter_mode = MCPWM_UP_COUNTER;
    // pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    setAngle(m_angle);
}

void Servo::setAngle(double angle) {
    m_angle = angle;
    // ESP_LOGI(TAG, "angle = %d", m_angle);
    // 古いメッセージを全て読み込んで空にする
    // ServoMessage msg;
    // while(m_xQueue != NULL && xQueueReceive(m_xQueue, (void*)&msg, (TickType_t)0) == pdTRUE)
    //     ;
    // 新しいメッセージキューをポスト
    ServoMessage msg = ServoMessage::Angle;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

void Servo::task(void* arg) {
    Servo* pThis = (Servo*)arg;
    ServoMessage msg;
    bool loop = true;
    while(loop) {
        // メツセージキュー読み取り
        if (pThis->m_xQueue != NULL && xQueueReceive(pThis->m_xQueue, (void*)&msg, portMAX_DELAY) == pdTRUE) {
            switch(msg) {
                case ServoMessage::Angle:   // サーボ制御
                    pThis->angle();
                    break;
            }
        }
    }
}

void Servo::angle() {
    // uint32_t angle = m_angle;
    // if (angle > MAX_DEGREE)
    //     angle = MAX_DEGREE;
    // uint32_t duty_us = MIN_PULSEWIDTH + (((MAX_PULSEWIDTH - MIN_PULSEWIDTH) * angle) / MAX_DEGREE);
    // ESP_LOGI(TAG, "angle angle = %d, duty_us = %lu", m_angle, (unsigned long)duty_us);
    // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_us);

    ESP_LOGI(TAG, "angle = %lf, duty_us = %lu", m_angle, example_angle_to_compare(m_angle));
    esp_err_t ret;
    if ((ret = mcpwm_comparator_set_compare_value(m_comparator, example_angle_to_compare(m_angle))) != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_comparator_set_compare_value error %d", ret);
    }
}