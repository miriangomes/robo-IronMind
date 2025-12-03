#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

/* ----------- CONFIGURAÇÃO DOS PINOS ----------- */
#define MOTOR_A_IN1 GPIO_NUM_13
#define MOTOR_A_IN2 GPIO_NUM_27
#define MOTOR_A_PWM GPIO_NUM_25

#define MOTOR_B_IN1 GPIO_NUM_14
#define MOTOR_B_IN2 GPIO_NUM_26
#define MOTOR_B_PWM GPIO_NUM_33

/* ----------- CONFIG VELOCIDADE (LEDC) ----------- */
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_A LEDC_CHANNEL_0
#define LEDC_CHANNEL_B LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY (5000)
#define MAX_SPEED 200 // Velocidade de teste (0 a 255)

/* ----------- INICIALIZAÇÃO DO HARDWARE ----------- */
void ledc_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Canal Motor A
    ledc_channel_config_t ledc_channel_a = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_A,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_A_PWM,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_a);

    // Canal Motor B
    ledc_channel_config_t ledc_channel_b = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_B,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_B_PWM,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_b);
}

void motores_init(void) {
    gpio_set_direction(MOTOR_A_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_A_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN2, GPIO_MODE_OUTPUT);
}

/* ----------- CONTROLE DE MOTORES ----------- */
void set_speed(int speed_a, int speed_b) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, speed_a);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, speed_b);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

void mov_frente() {
    printf("Motores: FRENTE\n");
    // Motor A
    gpio_set_level(MOTOR_A_IN1, 1); 
    gpio_set_level(MOTOR_A_IN2, 0);
    // Motor B
    gpio_set_level(MOTOR_B_IN1, 1); 
    gpio_set_level(MOTOR_B_IN2, 0);
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_tras() {
    printf("Motores: TRAS\n");
    // Motor A invertido
    gpio_set_level(MOTOR_A_IN1, 0); 
    gpio_set_level(MOTOR_A_IN2, 1);
    // Motor B invertido
    gpio_set_level(MOTOR_B_IN1, 0); 
    gpio_set_level(MOTOR_B_IN2, 1);
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_parada() {
    printf("Motores: PARADO\n");
    gpio_set_level(MOTOR_A_IN1, 0); 
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 0); 
    gpio_set_level(MOTOR_B_IN2, 0);
    set_speed(0, 0);
}

/* ----------- FUNÇÃO PRINCIPAL ----------- */
void app_main(void) {
    // 1. Inicializa drivers
    ledc_init();
    motores_init();
    
    printf("\n=== TESTE DE MOTORES INICIADO ===\n");
    printf("Aguardando 3 segundos de seguranca...\n");
    
    // 2. Aguarda 3 segundos antes de ligar qualquer coisa
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    printf("INICIANDO LOOP DE TESTE!\n");

    // 3. Loop infinito de teste
    while (1) {
        // Frente por 2s
        mov_frente();
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // Para por 1s
        mov_parada();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Trás por 2s
        mov_tras();
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // Para por 1s
        mov_parada();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}