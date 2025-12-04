#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Drivers
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

// Componentes do Sistema
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp32/rom/ets_sys.h"
#include "sdkconfig.h"

/* MODO DEBUG */
#define MODO_DEBUG 1
#if MODO_DEBUG
#define LOG(...) printf(__VA_ARGS__)
#else
#define LOG(...)
#endif

/* ----------- BOTÃO DE ESTRATÉGIA ----------- */
#define BOTAO_ESTRATEGIA GPIO_NUM_4  
#define TEMPO_DEBOUNCE_MS 300        

/* ----------- PARÂMETROS DE COMBATE ----------- */
#define DISTANCIA_ATAQUE 35.0 
#define TEMPO_RECUO_BORDA_MS 400
#define TEMPO_GIRO_BORDA_MS 250
#define DELAY_INICIAL_OBRIGATORIO_MS 3000 

/* ----------- MOTORES ----------- */
#define MOTOR_A_IN1 GPIO_NUM_13
#define MOTOR_A_IN2 GPIO_NUM_27
#define MOTOR_A_PWM GPIO_NUM_25

#define MOTOR_B_IN1 GPIO_NUM_14
#define MOTOR_B_IN2 GPIO_NUM_26
#define MOTOR_B_PWM GPIO_NUM_33

/* ----------- SENSORES ----------- */
#define SHARP_ESQ_CHANNEL ADC_CHANNEL_0
#define SHARP_DIR_CHANNEL ADC_CHANNEL_3 

// Sensores de linha (Digital Input)
#define SENSOR_LINHA_FRENTE GPIO_NUM_35
#define SENSOR_LINHA_TRAS GPIO_NUM_34

#define LINHA_DETECTADA 1

/* ----------- CONFIG VELOCIDADE ----------- */
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_A LEDC_CHANNEL_0
#define LEDC_CHANNEL_B LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY (5000)
#define MAX_SPEED 200

/* ----------- VARIÁVEIS GLOBAIS ----------- */
volatile int estrategia_selecionada = 1;
volatile uint32_t ultimo_clique = 0;
volatile bool combate_autorizado = false;  

adc_oneshot_unit_handle_t adc1_handle;

/* ----------- PROTÓTIPOS ----------- */
void mov_frente(void);
void mov_tras(void);
void mov_girar_esquerda(void);
void mov_girar_direita(void);
void mov_parada(void);
void estrategia_1_comum(float dist_esq, float dist_dir);
void estrategia_2_desviar(float dist_esq, float dist_dir);
void estrategia_3_desviar_atacar(float dist_esq, float dist_dir);

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

void botao_init(void) {
    gpio_set_direction(BOTAO_ESTRATEGIA, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOTAO_ESTRATEGIA, GPIO_PULLUP_ONLY);
}

void sensores_init(void) {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, 
        .atten = ADC_ATTEN_DB_12,        
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SHARP_ESQ_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SHARP_DIR_CHANNEL, &config));

    // --- SENSORES DE LINHA (DIGITAL) ---
    gpio_set_direction(SENSOR_LINHA_FRENTE, GPIO_MODE_INPUT);
    gpio_set_direction(SENSOR_LINHA_TRAS, GPIO_MODE_INPUT);
}

/* ----------- CONTROLE DE MOTORES ----------- */
void set_speed(int speed_a, int speed_b) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, speed_a);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, speed_b);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

void mov_frente() {
    if (!combate_autorizado) return;
    
    gpio_set_level(MOTOR_A_IN1, 1); 
    gpio_set_level(MOTOR_A_IN2, 0);
    
    gpio_set_level(MOTOR_B_IN1, 0); 
    gpio_set_level(MOTOR_B_IN2, 1);  
    
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_tras() {
    if (!combate_autorizado) return;
    gpio_set_level(MOTOR_A_IN1, 0); 
    gpio_set_level(MOTOR_A_IN2, 1);
    
    gpio_set_level(MOTOR_B_IN1, 1); 
    gpio_set_level(MOTOR_B_IN2, 0); 
    
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_girar_esquerda() {
    if (!combate_autorizado) return;
    
    gpio_set_level(MOTOR_A_IN1, 1); 
    gpio_set_level(MOTOR_A_IN2, 0);
    
    gpio_set_level(MOTOR_B_IN1, 1); 
    gpio_set_level(MOTOR_B_IN2, 0);
    
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_girar_direita() {
    if (!combate_autorizado) return;
    
    gpio_set_level(MOTOR_A_IN1, 0); 
    gpio_set_level(MOTOR_A_IN2, 1);
    
    gpio_set_level(MOTOR_B_IN1, 0); 
    gpio_set_level(MOTOR_B_IN2, 1);
    
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_parada() {
    gpio_set_level(MOTOR_A_IN1, 0); gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 0); gpio_set_level(MOTOR_B_IN2, 0);
    set_speed(0, 0);
}

/* ----------- LEITURA DE SENSORES ----------- */
float ler_sharp_cm(adc_channel_t canal) {
    int soma = 0;
    int leitura_raw = 0;
    const int NUM_LEITURAS = 5;
    
    for(int i = 0; i < NUM_LEITURAS; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, canal, &leitura_raw));
        soma += leitura_raw;
        ets_delay_us(200);
    }
    
    int sensorValue = soma / NUM_LEITURAS;
    
    if (sensorValue <= 9) return 999.0;
    
    float cmValue = (6762.0 / (float)(sensorValue - 9)) - 4.0;
    
    if (cmValue < 4.0 || cmValue > 80.0) return 999.0;
    
    return cmValue;
}

float ler_distancia_esquerda() {
    return ler_sharp_cm(SHARP_ESQ_CHANNEL); 
}

float ler_distancia_direita() {
    return ler_sharp_cm(SHARP_DIR_CHANNEL); 
}

/* ----------- LÓGICA DO BOTÃO ----------- */
void verificar_botao() {
    uint32_t tempo_atual = esp_timer_get_time() / 1000;
    
    if (gpio_get_level(BOTAO_ESTRATEGIA) == 0) {
        if (tempo_atual - ultimo_clique > TEMPO_DEBOUNCE_MS) {
            estrategia_selecionada++;
            if (estrategia_selecionada > 3) estrategia_selecionada = 1;
            
            LOG("\n*** ESTRATEGIA SELECIONADA: %d ***\n", estrategia_selecionada);
            ultimo_clique = tempo_atual;
            
            while(gpio_get_level(BOTAO_ESTRATEGIA) == 0) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }
}

/* ----------- ESTRATÉGIAS ----------- */
void estrategia_1_comum(float dist_esq, float dist_dir) {
    float dist_minima = (dist_esq < dist_dir) ? dist_esq : dist_dir;
    
    if (dist_minima < DISTANCIA_ATAQUE && dist_minima < 999) {
        // Inimigo perto: ATACAR
        mov_frente();
    } else {
        // Rastreamento
        if (dist_esq < dist_dir && dist_esq < 999) {
            mov_girar_esquerda();
        } else if (dist_dir < dist_esq && dist_dir < 999) {
            mov_girar_direita();
        } else {
            // Procura girando
            mov_girar_esquerda();
        }
    }
}

void estrategia_2_desviar(float dist_esq, float dist_dir) {
    static bool desviando = false;
    static uint32_t tempo_inicio_desvio = 0;
    float dist_minima = (dist_esq < dist_dir) ? dist_esq : dist_dir;
    
    if (dist_minima < DISTANCIA_ATAQUE && dist_minima < 999) {
        if (!desviando) {
            desviando = true;
            tempo_inicio_desvio = esp_timer_get_time() / 1000;
            // Desvia para o lado oposto ao que viu
            if (dist_esq < dist_dir) mov_girar_direita();
            else mov_girar_esquerda();
        } else {
            uint32_t tempo_atual = esp_timer_get_time() / 1000;
            if (tempo_atual - tempo_inicio_desvio > 400) {
                desviando = false;
                mov_frente(); // Ataca após desviar
            }
        }
    } else {
        desviando = false;
        // Rastreia se ver algo longe
        if (dist_esq < 999) mov_girar_esquerda();
        else if (dist_dir < 999) mov_girar_direita();
        else mov_girar_direita(); // Procura padrão
    }
}

void estrategia_3_desviar_atacar(float dist_esq, float dist_dir) {
    static int fase = 0;
    static uint32_t tempo_inicio_fase = 0;
    static bool direcao_desvio = false; // false=esq, true=dir
    
    uint32_t tempo_atual = esp_timer_get_time() / 1000;
    float dist_minima = (dist_esq < dist_dir) ? dist_esq : dist_dir;
    
    switch(fase) {
        case 0: // PROCURAR
            if (dist_minima < DISTANCIA_ATAQUE && dist_minima < 999) {
                // Viu inimigo, decide desvio
                direcao_desvio = (dist_esq < dist_dir) ? true : false; 
                fase = 1;
                tempo_inicio_fase = tempo_atual;
            } else {
                mov_girar_esquerda();
            }
            break;
        case 1: // DESVIAR DE LADO
            if (tempo_atual - tempo_inicio_fase < 300) {
                if (direcao_desvio) mov_girar_direita(); else mov_girar_esquerda();
            } else {
                fase = 2;
                tempo_inicio_fase = tempo_atual;
            }
            break;
        case 2: // CURVAR PARA VOLTAR (FLANK)
            if (tempo_atual - tempo_inicio_fase < 400) {
                if (direcao_desvio) mov_girar_esquerda(); else mov_girar_direita();
            } else {
                fase = 3;
                tempo_inicio_fase = tempo_atual;
            }
            break;
        case 3: // ATAQUE FINAL
            if (tempo_atual - tempo_inicio_fase < 1000) mov_frente();
            else fase = 0;
            break;
    }
}

/* ----------- FUNÇÃO PRINCIPAL ----------- */
void app_main(void) {
    ledc_init();
    motores_init();
    botao_init();
    sensores_init();
    
    LOG("\n=== ROBO INICIADO ===\n");
    LOG("Selecione estrategia (5s)...\n");
    
    uint32_t tempo_inicio_selecao = esp_timer_get_time() / 1000;
    
    // Loop de seleção de estratégia antes da luta
    while((esp_timer_get_time() / 1000 - tempo_inicio_selecao) < 5000) {
        verificar_botao();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    LOG("Delay OBRIGATORIO 5s...\n");
    mov_parada();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    combate_autorizado = true;
    LOG("=== COMBATE VALENDO ===\n");
    
    while (1) {
        // Leitura dos sensores
        float dist_esq = ler_distancia_esquerda();
        float dist_dir = ler_distancia_direita();
        int sensor_frente = gpio_get_level(SENSOR_LINHA_FRENTE);
        int sensor_tras = gpio_get_level(SENSOR_LINHA_TRAS);
        
        bool acao_borda = false;
        
        // --- PRIORIDADE 1: NÃO CAIR DO DOJO ---
        if (sensor_frente == LINHA_DETECTADA) {
            LOG("!!! BORDA FRENTE !!!\n");
            mov_tras();
            vTaskDelay(TEMPO_RECUO_BORDA_MS / portTICK_PERIOD_MS);
            mov_girar_direita();
            vTaskDelay(TEMPO_GIRO_BORDA_MS / portTICK_PERIOD_MS);
            acao_borda = true;
        }
        else if (sensor_tras == LINHA_DETECTADA) {
            LOG("!!! BORDA TRAS !!!\n");
            mov_frente();
            vTaskDelay(TEMPO_RECUO_BORDA_MS / portTICK_PERIOD_MS);
            acao_borda = true;
        }
        
        // --- PRIORIDADE 2: EXECUTAR ESTRATÉGIA ---
        if (!acao_borda) {
            switch(estrategia_selecionada) {
                case 1: estrategia_1_comum(dist_esq, dist_dir); break;
                case 2: estrategia_2_desviar(dist_esq, dist_dir); break;
                case 3: estrategia_3_desviar_atacar(dist_esq, dist_dir); break;
                default: estrategia_1_comum(dist_esq, dist_dir); break;
            }
        }
        
        // Pequeno delay para estabilidade do loop
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}