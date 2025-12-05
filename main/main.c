#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"
#include "esp32/rom/ets_sys.h"

/* ----------- MOTORES ----------- */
#define MOTOR_A_IN1 GPIO_NUM_27
#define MOTOR_A_IN2 GPIO_NUM_13
#define MOTOR_A_PWM GPIO_NUM_25

#define MOTOR_B_IN1 GPIO_NUM_26
#define MOTOR_B_IN2 GPIO_NUM_14
#define MOTOR_B_PWM GPIO_NUM_33

/* ----------- SENSORES ----------- */
#define SHARP_ESQ_CHANNEL ADC_CHANNEL_0  // GPIO36
#define SHARP_DIR_CHANNEL ADC_CHANNEL_3  // GPIO39
#define SENSOR_LINHA_FRENTE GPIO_NUM_35
#define SENSOR_LINHA_TRAS GPIO_NUM_34

/* ----------- CONFIG PWM ----------- */
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_A LEDC_CHANNEL_0
#define LEDC_CHANNEL_B LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY (5000)
#define VELOCIDADE 255

/* ----------- PARÂMETROS ----------- */
#define DISTANCIA_ATAQUE 40.0

adc_oneshot_unit_handle_t adc1_handle;

/* ----------- INIT ----------- */
static void ledc_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ch_a = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_A,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_A_PWM,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_a);

    ledc_channel_config_t ch_b = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_B,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_B_PWM,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_b);
}

static void motores_init(void) {
    gpio_reset_pin(MOTOR_A_IN1);
    gpio_reset_pin(MOTOR_A_IN2);
    gpio_reset_pin(MOTOR_B_IN1);
    gpio_reset_pin(MOTOR_B_IN2);
    gpio_set_direction(MOTOR_A_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_A_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN2, GPIO_MODE_OUTPUT);
}

static void sensores_init(void) {
    // Configuração ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,  
        .atten = ADC_ATTEN_DB_12      
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SHARP_ESQ_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SHARP_DIR_CHANNEL, &config));

    // Sensores de linha
    gpio_set_direction(SENSOR_LINHA_FRENTE, GPIO_MODE_INPUT);
    gpio_set_direction(SENSOR_LINHA_TRAS, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SENSOR_LINHA_FRENTE, GPIO_FLOATING);
    gpio_set_pull_mode(SENSOR_LINHA_TRAS, GPIO_FLOATING);
}

/* ----------- MOTORES ----------- */
static void set_speed(int a, int b) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, a);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, b);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

static void frente(void) {
    gpio_set_level(MOTOR_A_IN1, 1);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 1);
    gpio_set_level(MOTOR_B_IN2, 0);
    set_speed(VELOCIDADE, VELOCIDADE);
}

static void tras(void) {
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 1);
    gpio_set_level(MOTOR_B_IN1, 0);
    gpio_set_level(MOTOR_B_IN2, 1);
    set_speed(VELOCIDADE, VELOCIDADE);
}

static void girar_esquerda(void) {
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 1);
    gpio_set_level(MOTOR_B_IN1, 1);
    gpio_set_level(MOTOR_B_IN2, 0);
    set_speed(VELOCIDADE, VELOCIDADE);
}

static void girar_direita(void) {
    gpio_set_level(MOTOR_A_IN1, 1);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 0);
    gpio_set_level(MOTOR_B_IN2, 1);
    set_speed(VELOCIDADE, VELOCIDADE);
}

static void parado(void) {
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 0);
    gpio_set_level(MOTOR_B_IN2, 0);
    set_speed(0, 0);
}

/* ----------- SENSORES ----------- */
static float ler_sharp(adc_channel_t canal) {
    int valor;
    int soma = 0;
    int leituras_validas = 0;
    
    // Faz 10 leituras e pega a média
    for(int i = 0; i < 10; i++) {
        esp_err_t ret = adc_oneshot_read(adc1_handle, canal, &valor);
        if (ret == ESP_OK) {
            soma += valor;
            leituras_validas++;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    if (leituras_validas == 0) return 999.0;
    
    int media = soma / leituras_validas;
    
    printf("ADC Canal %d: raw=%d ", canal, media);
    
    if (media < 500) return 999.0;  
    
    float voltage = (media / 4095.0) * 3.3;
    float cm = 27.86 / (voltage - 0.42);
    
    printf("V=%.2f cm=%.1f\n", voltage, cm);
    
    if (cm < 10.0 || cm > 80.0) return 999.0;
    
    return cm;
}

/* ----------- MAIN ----------- */
void app_main(void) {
    ledc_init();
    motores_init();
    sensores_init();
    
    printf("\n=== ROBO INICIADO ===\n");
    printf("Delay 5s...\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    printf("=== COMBATE ===\n");
    
    while (1) {
        float dist_esq = ler_sharp(SHARP_ESQ_CHANNEL);
        float dist_dir = ler_sharp(SHARP_DIR_CHANNEL);
        int borda_frente = gpio_get_level(SENSOR_LINHA_FRENTE);
        int borda_tras = gpio_get_level(SENSOR_LINHA_TRAS);
        
        printf("Esq:%.1fcm Dir:%.1fcm Frente:%d Tras:%d\n", 
               dist_esq, dist_dir, borda_frente, borda_tras);
        
        // Prioridade 1: Não cair
        if (borda_frente == 0) {
            printf(">>> BORDA FRENTE!\n");
            tras();
            vTaskDelay(400 / portTICK_PERIOD_MS);
           // girar_direita();
           // vTaskDelay(250 / portTICK_PERIOD_MS);
        }
        else if (borda_tras == 0) {
            printf(">>> BORDA TRAS!\n");
            frente();
            vTaskDelay(400 / portTICK_PERIOD_MS);
        }
        // Prioridade 2: Atacar
        else {
            float menor = (dist_esq < dist_dir) ? dist_esq : dist_dir;
            
            if (menor < DISTANCIA_ATAQUE && menor < 999) {
                printf(">>> ATACAR!\n");
                frente();
            }
            else if (dist_esq < dist_dir && dist_esq < 999) {
                printf(">>> Giro esquerda\n");
                girar_esquerda();
            }
            else if (dist_dir < dist_esq && dist_dir < 999) {
                printf(">>> Giro direita\n");
                girar_direita();
            }
            else {
                printf(">>> Procurando...\n");
                girar_esquerda();
            }
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}