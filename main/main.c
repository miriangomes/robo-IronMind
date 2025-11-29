#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Drivers
#include "driver/gpio.h"
#include "driver/ledc.h"

// Componentes ESP-IDF
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
#define DISTANCIA_ATAQUE 15.0
#define TEMPO_RECUO_BORDA_MS 400
#define TEMPO_GIRO_BORDA_MS 250
#define DELAY_INICIAL_OBRIGATORIO_MS 5000  // 5 segundos obrigatórios

/* ----------- MOTORES ----------- */
#define MOTOR_A_IN1 GPIO_NUM_13
#define MOTOR_A_IN2 GPIO_NUM_27
#define MOTOR_A_PWM GPIO_NUM_25

#define MOTOR_B_IN1 GPIO_NUM_14
#define MOTOR_B_IN2 GPIO_NUM_26
#define MOTOR_B_PWM GPIO_NUM_33

/* ----------- SENSORES ----------- */
#define SENSOR_ULTRA_TRIGGER GPIO_NUM_36
#define SENSOR_ULTRA_ECHO GPIO_NUM_39
#define SENSOR_LINHA_FRENTE GPIO_NUM_35
#define SENSOR_LINHA_TRAS GPIO_NUM_34

/* ----------- CONFIG VELOCIDADE ----------- */
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_A LEDC_CHANNEL_0
#define LEDC_CHANNEL_B LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY (5000)
#define MAX_SPEED 255

/* ----------- VARIÁVEIS GLOBAIS ----------- */
volatile int estrategia_selecionada = 1;
volatile uint32_t ultimo_clique = 0;
volatile bool combate_autorizado = false;  

/* ----------- PROTÓTIPOS DAS FUNÇÕES ----------- */
void mov_frente(void);
void mov_tras(void);
void mov_girar_esquerda(void);
void mov_girar_direita(void);
void mov_parada(void);
void estrategia_1_comum(float distancia);
void estrategia_2_desviar(float distancia);
void estrategia_3_desviar_atacar(float distancia);

/* ----------- FUNÇÕES DE INICIALIZAÇÃO ----------- */
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
    gpio_set_direction(SENSOR_ULTRA_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_set_direction(SENSOR_ULTRA_ECHO, GPIO_MODE_INPUT);
    gpio_set_direction(SENSOR_LINHA_FRENTE, GPIO_MODE_INPUT);
    gpio_set_direction(SENSOR_LINHA_TRAS, GPIO_MODE_INPUT);
}

/* ----------- FUNÇÕES DE MOVIMENTO ----------- */
void set_speed(int speed_a, int speed_b) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, speed_a);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, speed_b);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

void mov_tras() {
    if (!combate_autorizado) return;  
    
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 1);
    gpio_set_level(MOTOR_B_IN1, 0);
    gpio_set_level(MOTOR_B_IN2, 1);
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_frente() {
    if (!combate_autorizado) return;  
    
    gpio_set_level(MOTOR_A_IN1, 1);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 1);
    gpio_set_level(MOTOR_B_IN2, 0);
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_girar_esquerda() {
    if (!combate_autorizado) return; 
    
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 1);
    gpio_set_level(MOTOR_B_IN1, 1);
    gpio_set_level(MOTOR_B_IN2, 0);
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_girar_direita() {
    if (!combate_autorizado) return;  
    
    gpio_set_level(MOTOR_A_IN1, 1);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 0);
    gpio_set_level(MOTOR_B_IN2, 1);
    set_speed(MAX_SPEED, MAX_SPEED);
}

void mov_parada() {
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 0);
    gpio_set_level(MOTOR_B_IN2, 0);
    set_speed(0, 0);
}

/* ----------- FUNÇÕES DOS SENSORES ----------- */
float ler_distancia_cm() {
    gpio_set_level(SENSOR_ULTRA_TRIGGER, 0);
    ets_delay_us(2);
    gpio_set_level(SENSOR_ULTRA_TRIGGER, 1);
    ets_delay_us(10);
    gpio_set_level(SENSOR_ULTRA_TRIGGER, 0);

    uint32_t startTime = esp_timer_get_time();
    while(gpio_get_level(SENSOR_ULTRA_ECHO) == 0) {
        if (esp_timer_get_time() - startTime > 500000) {
            return 999;
        }
    }
    
    startTime = esp_timer_get_time();
    uint32_t endTime = startTime;
    while(gpio_get_level(SENSOR_ULTRA_ECHO) == 1) {
        endTime = esp_timer_get_time();
        if (endTime - startTime > 500000) {
            return 999;
        }
    }

    long duration = endTime - startTime;
    return (float)(duration * 0.0343) / 2.0;
}

/* ----------- FUNÇÃO DE VERIFICAÇÃO DO BOTÃO ----------- */
void verificar_botao() {
    uint32_t tempo_atual = esp_timer_get_time() / 1000;
    
    if (gpio_get_level(BOTAO_ESTRATEGIA) == 0) {
        if (tempo_atual - ultimo_clique > TEMPO_DEBOUNCE_MS) {
            estrategia_selecionada++;
            if (estrategia_selecionada > 3) {
                estrategia_selecionada = 1;
            }
            
            LOG("\n*** ESTRATEGIA ALTERADA PARA: %d ***\n", estrategia_selecionada);
            ultimo_clique = tempo_atual;
            
            while(gpio_get_level(BOTAO_ESTRATEGIA) == 0) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }
}

/* ----------- ESTRATÉGIAS DE COMBATE ----------- */

void estrategia_1_comum(float distancia) {
    if (distancia < DISTANCIA_ATAQUE && distancia > 0) {
        LOG("    -> E1: ATACANDO! (dist=%.1f)\n", distancia);
        mov_frente();
    } else {
        LOG("    -> E1: Procurando... (dist=%.1f)\n", distancia);
        mov_girar_esquerda();
    }
}

void estrategia_2_desviar(float distancia) {
    static bool desviando = false;
    static uint32_t tempo_inicio_desvio = 0;
    
    if (distancia < DISTANCIA_ATAQUE && distancia > 0) {
        if (!desviando) {
            LOG("    -> E2: Desviando!\n");
            desviando = true;
            tempo_inicio_desvio = esp_timer_get_time() / 1000;
            
            if ((tempo_inicio_desvio % 2) == 0) {
                mov_girar_esquerda();
            } else {
                mov_girar_direita();
            }
        } else {
            uint32_t tempo_atual = esp_timer_get_time() / 1000;
            if (tempo_atual - tempo_inicio_desvio > 500) {
                desviando = false;
                mov_frente();
            }
        }
    } else {
        desviando = false;
        LOG("    -> E2: Procurando...\n");
        mov_girar_direita();
    }
}

void estrategia_3_desviar_atacar(float distancia) {
    static int fase = 0;
    static uint32_t tempo_inicio_fase = 0;
    
    uint32_t tempo_atual = esp_timer_get_time() / 1000;
    
    switch(fase) {
        case 0:  // PROCURAR
            if (distancia < DISTANCIA_ATAQUE && distancia > 0) {
                LOG("    -> E3: Iniciando manobra!\n");
                fase = 1;
                tempo_inicio_fase = tempo_atual;
            } else {
                LOG("    -> E3: Procurando...\n");
                mov_girar_esquerda();
            }
            break;
            
        case 1:  // DESVIAR
            if (tempo_atual - tempo_inicio_fase < 300) {
                mov_girar_direita();
            } else {
                LOG("    -> E3: Flanqueando...\n");
                fase = 2;
                tempo_inicio_fase = tempo_atual;
            }
            break;
            
        case 2:  // GIRAR
            if (tempo_atual - tempo_inicio_fase < 400) {
                mov_girar_esquerda();
            } else {
                LOG("    -> E3: ATAQUE!\n");
                fase = 3;
                tempo_inicio_fase = tempo_atual;
            }
            break;
            
        case 3:  // ATACAR
            if (tempo_atual - tempo_inicio_fase < 1000) {
                mov_frente();
            } else {
                fase = 0;
            }
            break;
    }
}

/* ----------- FUNÇÃO PRINCIPAL ----------- */
void app_main(void) {

    ledc_init();
    motores_init();
    botao_init();
    sensores_init();
    
    LOG("\n===========================================\n");
    LOG("    ROBO SUMO - SISTEMA DE ESTRATEGIAS\n");
    LOG("===========================================\n\n");
    
    mov_parada();
    combate_autorizado = false;  // Bloqueia movimento
    
    LOG("=== TESTE 1: SENSORES DE LINHA ===\n");
    LOG("Coloque o robo no centro (preto) e aguarde...\n\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    for(int i = 0; i < 10; i++) {
        int frente = gpio_get_level(SENSOR_LINHA_FRENTE);
        int tras = gpio_get_level(SENSOR_LINHA_TRAS);
        LOG("[Linha %02d] Frente=%d Tras=%d\n", i+1, frente, tras);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    
    LOG("\n=== TESTE 2: SENSOR ULTRASSONICO ===\n");
    LOG("Coloque um objeto a ~10cm e aguarde...\n\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    for(int i = 0; i < 10; i++) {
        float dist = ler_distancia_cm();
        LOG("[Ultra %02d] Distancia = %.2f cm", i+1, dist);
        
        if (dist < DISTANCIA_ATAQUE && dist > 0) {
            LOG(" -> DETECTARIA INIMIGO!\n");
        } else if (dist >= 999) {
            LOG(" -> ERRO/TIMEOUT\n");
        } else {
            LOG(" -> Muito longe\n");
        }
        
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    
    LOG("\n=== FIM DOS TESTES ===\n\n");
    
    // SELEÇÃO DE ESTRATÉGIA
    LOG("Aguardando selecao de estrategia (10s)...\n");
    LOG("Estrategia atual: %d\n", estrategia_selecionada);
    
    uint32_t tempo_inicio_selecao = esp_timer_get_time() / 1000;
    while((esp_timer_get_time() / 1000 - tempo_inicio_selecao) < 10000) {
        verificar_botao();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    LOG("\n*** ESTRATEGIA CONFIRMADA: %d ***\n\n", estrategia_selecionada);
    
    LOG("╔════════════════════════════════════════╗\n");
    LOG("║   DELAY OBRIGATORIO - ROBO PARADO     ║\n");
    LOG("║        Aguardando 5 segundos...       ║\n");
    LOG("╚════════════════════════════════════════╝\n\n");
    
    mov_parada();  
    
    for(int i = 5; i > 0; i--) {
        LOG("        >>> %d segundos... <<<\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    LOG("\n");
    LOG("╔════════════════════════════════════════╗\n");
    LOG("║      🔥 COMBATE AUTORIZADO! 🔥        ║\n");
    LOG("╚════════════════════════════════════════╝\n\n");
    
    combate_autorizado = true;
    
    while (1) {
        float distancia = ler_distancia_cm();
        int sensor_frente = gpio_get_level(SENSOR_LINHA_FRENTE);
        int sensor_tras = gpio_get_level(SENSOR_LINHA_TRAS);
        
        LOG("[LOOP] Dist=%.1fcm F=%d T=%d | ", distancia, sensor_frente, sensor_tras);
        
        bool acao_borda = false;
        
        // PRIORIDADE 1: BORDAS
        if (sensor_frente == 1) {  
            LOG("BORDA FRENTE!\n");
            mov_tras();
            vTaskDelay(TEMPO_RECUO_BORDA_MS / portTICK_PERIOD_MS);
            mov_girar_direita();
            vTaskDelay(TEMPO_GIRO_BORDA_MS / portTICK_PERIOD_MS);
            acao_borda = true;
        }
        else if (sensor_tras == 1) {
            LOG("BORDA TRAS!\n");
            mov_frente();
            vTaskDelay(TEMPO_RECUO_BORDA_MS / portTICK_PERIOD_MS);
            acao_borda = true;
        }
        
        // PRIORIDADE 2: ESTRATÉGIA
        if (!acao_borda) {
            LOG("Estrategia %d: ", estrategia_selecionada);
            
            switch(estrategia_selecionada) {
                case 1:
                    estrategia_1_comum(distancia);
                    break;
                case 2:
                    estrategia_2_desviar(distancia);
                    break;
                case 3:
                    estrategia_3_desviar_atacar(distancia);
                    break;
                default:
                    estrategia_1_comum(distancia);
                    break;
            }
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}