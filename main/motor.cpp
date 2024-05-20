#include <cmath>
#include "motor.hpp"
#include "esp_log.h"

#define TAG "Motor"

#define MOTOR_PWM_RESOLUTION_HZ 1000000       // 1MHz, 1us per tick
#define MOTOR_PWM_PERIOD 100                // 100 tick, 100μs = 10kHzのPWM周期

// メッセージ種別 (メッセージキュー用)
enum class MotorMessage {
    Speed           // 速度
};

Motor::Motor() {
    clear();
}

void Motor::clear() {
    m_xHandle = NULL;
    m_xQueue = NULL;
    m_gpioINA = (gpio_num_t)0;
    m_gpioINB = (gpio_num_t)0;
    m_md = MotorDirection::Stop;
    m_timer = NULL;
    m_oper = NULL;
    m_comparator = NULL;
    m_generator = NULL;
    m_speed = 0;
}

void Motor::init(int groupID, gpio_num_t gpioINA, gpio_num_t gpioINB) {
    ESP_LOGI(TAG, "init(S)");

    m_groupID = groupID;
    m_gpioINA = gpioINA;
    m_gpioINB = gpioINB;

    // タスク作成
    xTaskCreate(Motor::task, TAG, configMINIMAL_STACK_SIZE * 2, (void*)this, 1, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(MotorMessage));

    // PWM初期化
    ESP_LOGI(TAG, "init(1)");
    m_timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = m_groupID,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MOTOR_PWM_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = MOTOR_PWM_PERIOD,
    };
    esp_err_t ret = mcpwm_new_timer(&timer_config, &m_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_timer error %d", ret);
        return;
    }

    ESP_LOGI(TAG, "init(2)");
    m_oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = m_groupID,
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

    ESP_LOGI(TAG, "init(3)");
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

    ESP_LOGI(TAG, "init(4)");
    m_generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = m_gpioINA,
    };
    ret = mcpwm_new_generator(m_oper, &generator_config, &m_generator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_generator error %d", ret);
        return;
    }
    if (m_generator == NULL) {
        ESP_LOGE(TAG, "mcpwm_new_generator error");
    }

    ESP_LOGI(TAG, "init(5) m_speed=%d", m_speed);
    ret = mcpwm_comparator_set_compare_value(m_comparator, (uint32_t)0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_comparator_set_compare_value error %d", ret);
        return;
    }

    ESP_LOGI(TAG, "init(6)");
    ret = mcpwm_generator_set_action_on_timer_event(m_generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_generator_set_actions_on_timer_event error %d", ret);
        return;
    }
    ESP_LOGI(TAG, "init(7)");
    ret = mcpwm_generator_set_action_on_compare_event(m_generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_comparator, MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_generator_set_actions_on_compare_event error %d", ret);
        return;
    }
    
    ESP_LOGI(TAG, "init(8)");
    ret = mcpwm_timer_enable(m_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_enable error %d", ret);
        return;
    }
    ESP_LOGI(TAG, "init(9)");
    ret = mcpwm_timer_start_stop(m_timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_start_stop error %d", ret);
        return;
    }

    // INB初期化
    ESP_LOGI(TAG, "init(10)");
    gpio_reset_pin(m_gpioINB);
    gpio_set_direction(m_gpioINB, GPIO_MODE_OUTPUT);

    setSpeed(m_speed);

    ESP_LOGI(TAG, "init(E)");
}

void Motor::setDirection(MotorDirection md) {
    m_md = md;
}

void Motor::setSpeed(int speed) {
    m_speed = speed;    
    // 新しいメッセージキューをポスト
    MotorMessage msg = MotorMessage::Speed;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

void Motor::task(void* arg) {
    Motor* pThis = (Motor*)arg;
    MotorMessage msg;
    bool loop = true;
    while(loop) {
        // メツセージキュー読み取り
        if (pThis->m_xQueue != NULL && xQueueReceive(pThis->m_xQueue, (void*)&msg, portMAX_DELAY) == pdTRUE) {
            switch(msg) {
                case MotorMessage::Speed:       // 速度
                    pThis->speed();
                    break;
            }
        }
    }
}

void Motor::speed() {
    esp_err_t ret;
    int speed1 = 0;
    int speed2 = 0;
    switch(m_md) {
        case MotorDirection::Foward:    // 前進
            speed1 = m_speed;
            speed2 = 0;
            break;
        case MotorDirection::Back:      // 後退
            speed1 = m_speed;
            speed2 = 1;
            break;
        case MotorDirection::Stop:      // 停止
            speed1 = 0;
            speed2 = 0;
            break;
        case MotorDirection::Brake:     // ブレーキ
            speed1 = 0;
            speed2 = 0;
            break;
    }
    gpio_set_level(m_gpioINB, speed2);
    if ((ret = mcpwm_comparator_set_compare_value(m_comparator, (uint32_t)speed1)) != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_comparator_set_compare_value error %d", ret);
    }
}
