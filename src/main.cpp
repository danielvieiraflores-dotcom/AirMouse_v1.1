/*
 TO DOS a melhorar (13set2025)
  - ao usar os modos de orientação, às vezes, ele não reconhece a posição (ex. coloca na vertical e ele ainda pensa que está na horizontal)
  - Percebe-se que a posição de montagem do MPU está indicando posição ESQUERDA, quando está inclinado para DIREITA - e vice-versa.
  - Não funciona na posição LATERAL
*/

/*

 ---16set2025 - (Update para eliminar o Wrist Tilt. Ainda não foi testado)

 --- 31ago2025 - Daniel Flores (VERSÃO COM ORIENTAÇÃO AUTOMÁTICA + FILTROS AVANÇADOS)
 Aplicação: Movimento de Mouse e Teclado via MPU-6050 (Modo Laser Pointer)
 **V5 COMPLETA - ESP32 VERSION**

 - Lê dados do sensor GY-521 (MPU-6050) via I2C.
 - Usa a velocidade angular do giroscópio para controlar a velocidade do cursor.
 - Lê o estado de um botão conectado ao GPIO 15.
 - Envia os dados (velocidade X, velocidade Y, estado do botão) via comunicação serial no formato "X:0.000 Y:0.000 B:0".
 - Mantém a função de autocalibração do giroscópio para anular o drift quando o dispositivo está parado.
 - Comunicação I2C robusta, filtro DLPF em 42Hz, dead zone inteligente, smoothing corrigido e **DETECÇÃO AUTOMÁTICA DE ORIENTAÇÃO**.
 - **NOVO**: Detecta automaticamente como o usuário está segurando o dispositivo e adapta os controles.
 - **FILTROS AVANÇADOS**: Dead zone inteligente, zonas adaptivas, sensibilidade dinâmica, anti-jerk e smoothing otimizado.

 Um script Python (gyro_mouseCOMbotao.py) é necessário para interpretar esses dados seriais e convertê-los em comandos de mouse no computador.
*/
#include <Arduino.h>
#include <Wire.h>

// ---------------- Pinagem ESP32 -------------------
const int BUTTON_PIN = 15;    // GPIO 15 para o botão
const int SDA_PIN = 21;       // GPIO 21 para SDA (I2C)
const int SCL_PIN = 22;       // GPIO 22 para SCL (I2C)

// ---------------- Configuração geral ----------------
const int SAMPLE_HZ = 100; // Freq. amostragem alvo

// ---------------- MPU-6050 ----------------
const uint8_t MPU_ADDR         = 0x68;
const uint8_t MPU_WHO_AM_I_EXPECTED = 0x70; // 0x68; original
const uint8_t REG_CONFIG       = 0x1A; // DLPF
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

const float ACCEL_SENSITIVITY = 16384.0f; // ±2g
const float GYRO_SENSITIVITY  = 131.0f;   // ±250 dps

// Offsets de giroscópio (atualizados pela calibração)
float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;

// ---------------- Auto-calibração ----------------
const unsigned long CALIBRATION_TIME_MS = 2000;
const float STATIONARY_THRESHOLD_ACC_G = 0.08f;
const float STATIONARY_THRESHOLD_GYRO_DPS = 1.5f;

unsigned long stationaryStartTime = 0;
bool wasStationary = false;

// ---------------- Estado ----------------
unsigned long lastMicros = 0;

// ---------------- Filtros Avançados ----------------
// Dead zones por zona de sensibilidade
const float DEADZONE_THRESHOLD = 0.8f;        // Threshold principal
const float PRECISION_ZONE = 3.0f;            // Zona de precisão
const float NORMAL_ZONE = 8.0f;               // Zona normal

// Sensibilidades adaptivas
const float PRECISION_SENSITIVITY = 0.4f;     // Movimentos lentos
const float NORMAL_SENSITIVITY = 0.8f;        // Movimentos normais  
const float FAST_SENSITIVITY = 1.2f;          // Movimentos rápidos

// Parâmetros de smoothing (CORRIGIDOS)
const float SMOOTHING_ALPHA = 0.75f;          // 0.0 = sem filtro, 1.0 = máximo filtro
const float JERK_LIMIT = 4.0f;                // Limite para mudanças bruscas por sample

// Estados do filtro
float smoothed_x = 0.0f;
float smoothed_y = 0.0f;
float prev_output_x = 0.0f;
float prev_output_y = 0.0f;

// Sensibilidade adaptiva atual
float current_sens_x = NORMAL_SENSITIVITY;
float current_sens_y = NORMAL_SENSITIVITY;

// ---------------- SISTEMA DE ORIENTAÇÃO AUTOMÁTICA ----------------
// Tipos de orientação detectados
enum DeviceOrientation {
    ORIENTATION_HORIZONTAL = 0,    // Como controle remoto (padrão)
    ORIENTATION_VERTICAL = 1,      // Como smartphone vertical
    ORIENTATION_INVERTED = 2,      // De cabeça para baixo
    ORIENTATION_LEFT = 3,          // Inclinado para esquerda
    ORIENTATION_RIGHT = 4,         // Inclinado para direita
    ORIENTATION_UNKNOWN = 5        // Não identificada
};

// Estado atual da orientação
DeviceOrientation currentOrientation = ORIENTATION_UNKNOWN;
DeviceOrientation lastStableOrientation = ORIENTATION_HORIZONTAL;

// Parâmetros de detecção
const float ORIENTATION_THRESHOLD = 0.6f;    // Mínimo para detectar orientação (0.6g)
const unsigned long ORIENTATION_STABLE_TIME = 1500; // 1.5s para confirmar orientação
unsigned long orientationChangeTime = 0;

// Estrutura para mapeamento de orientação
struct OrientationMapping {
    float pitch_factor;   // Multiplicador para movimento horizontal (pitch)
    float roll_factor;    // Multiplicador para movimento vertical (roll)  
    float yaw_factor;     // Multiplicador para yaw
    bool invert_x;        // Inverter X final
    bool invert_y;        // Inverter Y final
    const char* name;     // Nome da orientação
};

// Mapeamentos para cada orientação
const OrientationMapping orientationMaps[] = {
    // HORIZONTAL (padrão - como controle remoto)
    { 1.0f,  1.0f,  1.0f, false, false, "HORIZONTAL" },

    // VERTICAL (como smartphone em pé)
    { 1.0f,  1.0f,  1.0f, true,  false, "VERTICAL" },

    // INVERTED (de cabeça para baixo)
    { 1.0f,  1.0f,  1.0f, true,  true,  "INVERTIDO" },
    
    // LEFT (inclinado 90° esquerda)
    { -1.0f, 1.0f,  1.0f, false, true,  "ESQUERDA" },
    
    // RIGHT (inclinado 90° direita) 
    { 1.0f, -1.0f,  1.0f, false, false, "DIREITA" },
    
    // UNKNOWN (usar padrão)
    { -1.0f,  1.0f,  1.0f, false, false, "DESCONHECIDA" }
};

// Parâmetros para reduzir a influência de inclinações involuntárias do punho no movimento horizontal
const float WRIST_TILT_SUPPRESSION_GAIN      = 0.65f;  // Intensidade da redução do movimento horizontal
const float WRIST_TILT_SUPPRESSION_MIN_SCALE = 0.35f;  // Limite mínimo para preservar algum movimento
const float WRIST_TILT_DEADZONE              = 1.0f;   // Ignora pequenas inclinações (em dps)
const float WRIST_TILT_EPSILON               = 0.001f; // Evita divisões por zero

// ---------------- FUNÇÕES DE FILTRO AVANÇADO ----------------

// Dead zone inteligente com transição suave
float smartDeadZone(float input, float threshold) {
    float abs_input = fabs(input);
    
    if (abs_input <= threshold) {
        return 0.0f;
    }
    
    // Transição suave para fora da dead zone
    float excess = abs_input - threshold;
    float smooth_factor = excess / threshold; // Fator de transição
    smooth_factor = constrain(smooth_factor, 0.0f, 2.0f);
    
    return (input > 0) ? excess * smooth_factor : -excess * smooth_factor;
}

// Calcula sensibilidade adaptiva baseada na magnitude do movimento
float calculateAdaptiveSensitivity(float input, float current_sens) {
    float abs_input = fabs(input);
    float target_sens;
    
    if (abs_input <= PRECISION_ZONE) {
        target_sens = PRECISION_SENSITIVITY;
    } else if (abs_input <= NORMAL_ZONE) {
        target_sens = NORMAL_SENSITIVITY;
    } else {
        target_sens = FAST_SENSITIVITY;
    }
    
    // Transição suave entre sensibilidades
    return current_sens * 0.9f + target_sens * 0.1f;
}

// Limita mudanças bruscas (anti-jerk)
float limitJerk(float current, float previous, float max_change) {
    float change = current - previous;
    if (fabs(change) > max_change) {
        return previous + (change > 0 ? max_change : -max_change);
    }
    return current;
}

// Processamento completo de movimento
void processMovement(float raw_x, float raw_y, float* output_x, float* output_y) {
    // 1. Aplicar dead zone inteligente
    float deadzone_x = smartDeadZone(raw_x, DEADZONE_THRESHOLD);
    float deadzone_y = smartDeadZone(raw_y, DEADZONE_THRESHOLD);
    
    // 2. Calcular sensibilidade adaptiva
    current_sens_x = calculateAdaptiveSensitivity(deadzone_x, current_sens_x);
    current_sens_y = calculateAdaptiveSensitivity(deadzone_y, current_sens_y);
    
    // 3. Aplicar sensibilidade
    float scaled_x = deadzone_x * current_sens_x;
    float scaled_y = deadzone_y * current_sens_y;
    
    // 4. Limitar jerk (mudanças bruscas)
    float jerk_limited_x = limitJerk(scaled_x, prev_output_x, JERK_LIMIT);
    float jerk_limited_y = limitJerk(scaled_y, prev_output_y, JERK_LIMIT);
    
    // 5. Aplicar smoothing (FÓRMULA CORRIGIDA)
    smoothed_x = smoothed_x * SMOOTHING_ALPHA + jerk_limited_x * (1.0f - SMOOTHING_ALPHA);
    smoothed_y = smoothed_y * SMOOTHING_ALPHA + jerk_limited_y * (1.0f - SMOOTHING_ALPHA);
    
    // 6. Atualizar estados anteriores
    prev_output_x = jerk_limited_x;
    prev_output_y = jerk_limited_y;
    
    // 7. Retornar valores processados
    *output_x = smoothed_x;
    *output_y = smoothed_y;
}

// Reset dos filtros após calibração
void resetFilters() {
    smoothed_x = 0.0f;
    smoothed_y = 0.0f;
    prev_output_x = 0.0f;
    prev_output_y = 0.0f;
    current_sens_x = NORMAL_SENSITIVITY;
    current_sens_y = NORMAL_SENSITIVITY;
}

// ---------------- FUNÇÕES DE ORIENTAÇÃO ----------------

// Detecta orientação baseada no acelerômetro
DeviceOrientation detectOrientation(float ax, float ay, float az) {
    // Valores absolutos para facilitar detecção
    float abs_ax = fabs(ax);
    float abs_ay = fabs(ay);
    float abs_az = fabs(az);
    
    // Encontrar o eixo dominante (onde a gravidade está atuando mais)
    
    // Z dominante (normal/invertido)
    if (abs_az > ORIENTATION_THRESHOLD && abs_az >= abs_ax && abs_az >= abs_ay) {
        if (az > 0) {
            return ORIENTATION_HORIZONTAL;  // Normal (Z para cima)
        } else {
            return ORIENTATION_INVERTED;    // Invertido (Z para baixo)
        }
    }
    // Y dominante (vertical como smartphone)
    else if (abs_ay > ORIENTATION_THRESHOLD && abs_ay >= abs_ax && abs_ay >= abs_az) {
        return ORIENTATION_VERTICAL;
    }
    // X dominante (inclinado lateralmente)
    else if (abs_ax > ORIENTATION_THRESHOLD && abs_ax >= abs_ay && abs_ax >= abs_az) {
        if (ax > 0) {
            return ORIENTATION_RIGHT;   // Inclinado para direita
        } else {
            return ORIENTATION_LEFT;    // Inclinado para esquerda
        }
    }
    
    return ORIENTATION_UNKNOWN;
}

// Atualiza orientação com estabilização temporal
void updateOrientation(float ax, float ay, float az) {
    DeviceOrientation detected = detectOrientation(ax, ay, az);
    
    // Se detectou orientação diferente
    if (detected != currentOrientation) {
        if (orientationChangeTime == 0) {
            // Primeira detecção de mudança
            orientationChangeTime = millis();
            currentOrientation = detected;
        } else if (millis() - orientationChangeTime > ORIENTATION_STABLE_TIME) {
            // Mudança estável por tempo suficiente - confirmar
            if (detected < ORIENTATION_UNKNOWN) { // Só aceitar orientações válidas
                lastStableOrientation = detected;
                Serial.print("I:ORIENTACAO_"); 
                Serial.println(orientationMaps[detected].name);
                orientationChangeTime = 0;
            }
        }
    } else {
        // Orientação voltou ao normal - resetar timer
        orientationChangeTime = 0;
    }
}

// Atenua o impacto da inclinação do punho sobre o movimento horizontal
float suppressWristTilt(float horizontalValue, float roll_movement, const OrientationMapping& map) {
    float tilt_component = fabs(roll_movement * map.roll_factor);
    if (tilt_component <= WRIST_TILT_DEADZONE) {
        return horizontalValue;
    }

    tilt_component -= WRIST_TILT_DEADZONE; // remove a zona morta para pequenas inclinações

    float horizontal_magnitude = fabs(horizontalValue) + WRIST_TILT_EPSILON;
    float tilt_ratio = tilt_component / (tilt_component + horizontal_magnitude);

    float suppression = 1.0f - tilt_ratio * WRIST_TILT_SUPPRESSION_GAIN;
    suppression = constrain(suppression, WRIST_TILT_SUPPRESSION_MIN_SCALE, 1.0f);

    return horizontalValue * suppression;
}

// Aplica correção de orientação aos movimentos do mouse
void applyOrientationCorrection(float pitch_movement, float roll_movement, float yaw_movement,
                               float* corrected_horizontal, float* corrected_vertical) {

    const OrientationMapping& map = orientationMaps[lastStableOrientation];

    // Aplicar fatores de rotação baseados na orientação
    float temp_horizontal = pitch_movement * map.pitch_factor + yaw_movement * map.yaw_factor;
    float temp_vertical = roll_movement * map.roll_factor;

    // Reduzir influência de inclinações involuntárias do punho
    temp_horizontal = suppressWristTilt(temp_horizontal, roll_movement, map);

    // Aplicar inversões se necessário
    *corrected_horizontal = map.invert_x ? -temp_horizontal : temp_horizontal;
    *corrected_vertical = map.invert_y ? -temp_vertical : temp_vertical;
}

// ---------------- I2C helpers ----------------
bool mpuWrite(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission(true) == 0;
}

bool safeMpuRead(uint8_t reg, uint8_t n, uint8_t* buf) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    size_t bytesRead = Wire.requestFrom(static_cast<uint8_t>(MPU_ADDR), static_cast<size_t>(n), true);
    if (bytesRead != n) {
        return false;
    }

    Wire.readBytes(buf, n);
    return true;
}

bool setupMPU() {
    if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) {   // Tira do modo sleep
        return false;
    }
    if (!mpuWrite(REG_CONFIG, 0x03)) {       // Define DLPF para 42Hz (Gyro)
        return false;
    }
    if (!mpuWrite(REG_ACCEL_CONFIG, 0x00)) { // Acelerômetro em ±2g
        return false;
    }
    if (!mpuWrite(REG_GYRO_CONFIG,  0x00)) { // Giroscópio em ±250 dps
        return false;
    }
    return true;
}

bool recoverFromI2CFailure() {
    Serial.println("I:RECUPERANDO_I2C");
    Wire.end();
    delay(50);
    Wire.begin(SDA_PIN, SCL_PIN);  // ESP32 specific I2C initialization
    delay(10);

    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (setupMPU()) {
            uint8_t who = 0;
            if (safeMpuRead(REG_WHO_AM_I, 1, &who) && who == MPU_WHO_AM_I_EXPECTED) {
                return true;
            }
        }
        delay(100);
    }

    Serial.println("E4:RECUPERACAO_I2C_FALHOU");
    return false;
}

bool waitForMPU(uint32_t timeout_ms) {
    unsigned long start = millis();
    do {
        Wire.beginTransmission(MPU_ADDR);
        if (Wire.endTransmission(true) == 0) {
            uint8_t who = 0;
            if (safeMpuRead(REG_WHO_AM_I, 1, &who) && who == MPU_WHO_AM_I_EXPECTED) {
                return true;
            }
        } else {
            Serial.println("E1:MPU_NAO_ENCONTRADO. Tentando recuperar...");
            recoverFromI2CFailure();
        }
        delay(200);
    } while (millis() - start < timeout_ms);

    return false;
}

void calibrateGyro(uint16_t samples = 800) {
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint8_t buf[14];

    Serial.println("Calibrando giroscopio... Mantenha o dispositivo parado.");
    for (uint16_t i = 0; i < samples; i++) {
        if (!safeMpuRead(REG_ACCEL_XOUT_H, 14, buf)) {
            Serial.println("E2:LEITURA_FALHOU (calibracao)");
            if (!recoverFromI2CFailure()) {
                Serial.println("E5:RECUPERACAO_FALHOU (calibracao)");
            }
            i = 0;
            sum_gx = 0;
            sum_gy = 0;
            sum_gz = 0;
            delay(100);
            continue;
        }
        int16_t gx = (buf[8] << 8) | buf[9];
        int16_t gy = (buf[10] << 8) | buf[11];
        int16_t gz = (buf[12] << 8) | buf[13];
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        delay(3);
    }

    gx_offset = (sum_gx / (float)samples) / GYRO_SENSITIVITY;
    gy_offset = (sum_gy / (float)samples) / GYRO_SENSITIVITY;
    gz_offset = (sum_gz / (float)samples) / GYRO_SENSITIVITY;
    
    resetFilters(); // IMPORTANTE: Reset dos filtros após calibração
    Serial.println("Calibracao completa - Filtros resetados.");
}

void checkAutoCalibration(float ax, float ay, float az, float gx_raw, float gy_raw, float gz_raw) {
    float accelMagnitude = sqrtf(ax*ax + ay*ay + az*az);
    float gyroMagnitude  = sqrtf(gx_raw*gx_raw + gy_raw*gy_raw + gz_raw*gz_raw);

    bool isStationary = (fabsf(accelMagnitude - 1.0f) < STATIONARY_THRESHOLD_ACC_G) &&
                        (gyroMagnitude < STATIONARY_THRESHOLD_GYRO_DPS);

    if (isStationary && !wasStationary) {
        stationaryStartTime = millis();
        wasStationary = true;
    } else if (!isStationary) {
        wasStationary = false;
    }

    if (wasStationary && (millis() - stationaryStartTime > CALIBRATION_TIME_MS)) {
        calibrateGyro(500);
        stationaryStartTime = millis();
    }
}

// ---------------- Setup e Loop ----------------
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // ESP32 specific I2C initialization with defined pins
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);

    Serial.println("=== GYRO MOUSE COM ORIENTACAO AUTOMATICA + FILTROS AVANCADOS - ESP32 VERSION ===");
    Serial.println("Pinout ESP32:");
    Serial.print("  SDA: GPIO "); Serial.println(SDA_PIN);
    Serial.print("  SCL: GPIO "); Serial.println(SCL_PIN);
    Serial.print("  BUTTON: GPIO "); Serial.println(BUTTON_PIN);
    Serial.println("Procurando MPU-6050...");
    
    while (!waitForMPU(2000)) {
        Serial.println("E1:MPU_NAO_RESPONDE. Verifique conexoes e alimentacao.");
        delay(2000);
    }
    Serial.println("MPU-6050 encontrado!");

    if (!setupMPU()) {
        Serial.println("E4:CONFIG_INICIAL_FALHOU. Tentando recuperar...");
        if (!recoverFromI2CFailure()) {
            Serial.println("E4:CONFIG_NAO_RECUPERADA. Reinicie o dispositivo.");
            while (true) {
                delay(1000);
            }
        }
    }
    delay(100);

    calibrateGyro();
    
    Serial.println("Sistema iniciado! Orientacao sera detectada automaticamente.");
    Serial.println("Orientacoes suportadas: HORIZONTAL, VERTICAL, INVERTIDO, ESQUERDA, DIREITA");
    Serial.println("Filtros avancados ativados: Dead zone inteligente, sensibilidade adaptiva, anti-jerk, smoothing otimizado");

    lastMicros = micros();
}

void loop() {
    unsigned long now = micros();
    if (now - lastMicros < 1000000UL / SAMPLE_HZ) return;
    lastMicros = now;

    uint8_t buf[14];
    if (!safeMpuRead(REG_ACCEL_XOUT_H, 14, buf)) {
        Serial.println("E2:LEITURA_FALHOU");
        if (!recoverFromI2CFailure()) {
            Serial.println("E5:RECUPERACAO_FALHOU");
        }
        return;
    }

    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];

    float fax = ax / ACCEL_SENSITIVITY;
    float fay = ay / ACCEL_SENSITIVITY;
    float faz = az / ACCEL_SENSITIVITY;

    float gx_raw = gx / GYRO_SENSITIVITY;
    float gy_raw = gy / GYRO_SENSITIVITY;
    float gz_raw = gz / GYRO_SENSITIVITY;

    if (isnan(fax) || isinf(fax) || isnan(fay) || isinf(fay) || isnan(faz) || isinf(faz) ||
        isnan(gx_raw) || isinf(gx_raw) || isnan(gy_raw) || isinf(gy_raw) || isnan(gz_raw) || isinf(gz_raw)) {
        Serial.println("E3:DADOS_INVALIDOS");
        setupMPU();
        return;
    }

    // Atualizar detecção de orientação usando acelerômetro
    updateOrientation(fax, fay, faz);

    checkAutoCalibration(fax, fay, faz, gx_raw, gy_raw, gz_raw);

    // Calcular movimentos básicos do giroscópio (com offsets removidos)
    float pitch_movement = gy_raw - gy_offset; // Pitch (movimento horizontal base)
    float roll_movement = gx_raw - gx_offset;  // Roll (movimento vertical)
    float yaw_movement = gz_raw - gz_offset;   // Yaw (rotação adicional)

    // Aplicar correção automática de orientação
    float horizontal_corrected, vertical_corrected;
    applyOrientationCorrection(pitch_movement, roll_movement, yaw_movement, 
                              &horizontal_corrected, &vertical_corrected);

    int buttonState = digitalRead(BUTTON_PIN);
    int buttonValue = (buttonState == LOW) ? 1 : 0;

    // **NOVO**: Aplicar processamento avançado de filtros
    float processed_x, processed_y;
    processMovement(-horizontal_corrected, -vertical_corrected, &processed_x, &processed_y);

    // Enviar dados processados
    
    Serial.print("X:"); Serial.print(processed_x, 3);
    Serial.print(" Y:"); Serial.print(processed_y, 3);
    Serial.print(" B:"); Serial.print(buttonValue);
    Serial.print("\n");
    

    // Debug opcional (descomente para ver adaptação em tempo real)
 /*   
    if (millis() % 3000 < 50) {
        Serial.print("DEBUG - SensX:"); Serial.print(current_sens_x, 2);
        Serial.print(" SensY:"); Serial.print(current_sens_y, 2);
        Serial.print(" Orientacao:"); Serial.print(orientationMaps[lastStableOrientation].name);
        Serial.print(" RawMag:"); Serial.print(sqrt(horizontal_corrected*horizontal_corrected + vertical_corrected*vertical_corrected), 2);
        Serial.println();
    }
 */  
}
