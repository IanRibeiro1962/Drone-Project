// Config.h - Configurações e Definições Globais (VERSÃO CORRIGIDA)
#ifndef CONFIG_H
#define CONFIG_H

#include <Wire.h>
#include "Estruturas.h"

// ============================================================================
// CONSTANTES DO SISTEMA
// ============================================================================

// Temporização
const float CONTROL_TS = 0.01f;        // 10ms = 100Hz (período de controle)
const int sampleCount = 2000;          // Amostras para Allan Variance

// Limites de Segurança
const float MAX_START_ANGLE = 15.0f;    // Ângulo máximo para inicialização
const float MAX_ANGLE_OUTPUT = 100.0f;  // °/s máximo para setpoint do rate
const float MAX_RATE_OUTPUT = 400.0f;   // Limite para saída do rate loop

// ============================================================================
// CONFIGURAÇÃO DOS ESCs/MOTORES
// ============================================================================

// Valores PWM para ESCs (50Hz, 16 bits - ESP32)
const uint16_t ESC_PULSE_1MS = 3276;    // 1ms = (1000/20000) * 65535
const uint16_t ESC_PULSE_2MS = 6553;    // 2ms = (2000/20000) * 65535
const uint16_t ESC_PULSE_MIN = ESC_PULSE_1MS;
const uint16_t ESC_PULSE_MAX = ESC_PULSE_2MS;

const int16_t COMPENSACAO_MOTOR = 5; // Compensação para equalizar motores (ajuste fino)
const uint16_t THROTTLE_MINIMO = 3510; // Throttle mínimo baseado no teste (7%)

// Tempos de calibração (ms)
const uint16_t CALIB_DELAY_HIGH = 3000; // Pulso alto na calibração
const uint16_t CALIB_DELAY_LOW = 3000;  // Pulso baixo na calibração

// Pinos dos Motores - VERIFICAR NO SEU HARDWARE!
const int motorsPins[] = { 18, 19 };
const int numChannels = sizeof(motorsPins) / sizeof(motorsPins[0]);

// ============================================================================
// DECLARAÇÕES DE VARIÁVEIS GLOBAIS
// (Definidas no Balancin.ino)
// ============================================================================

// Arrays para análise de dados
extern float AccDataXZ[];
extern float AccDataYZ[];
extern float allanVarXZ, allanVarYZ;

// Variáveis de tempo
extern float Ts, lastTime, currentTime, Ts_calib;
extern unsigned long lastControlTime;

// Dados do sensor MPU6050
extern float RateRoll, RatePitch, RateYaw;
extern float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
extern float AccX, AccY, AccZ;
extern float AngleRoll, AnglePitch;

// Variáveis de controle
extern float InputRoll, InputThrottle;
extern float DesiredAngleRoll, DesiredRateRoll;
extern float ErrorAngleRoll, ErrorRateRoll;

// Saídas do controle em cascata
extern float saidaAngulo, saidaTaxa;

// Filtro de Kalman
extern float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
extern float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
extern float Kalman1DOutput[2];

// Parâmetros PID
extern float PAngleRoll, IAngleRoll, DAngleRoll;
extern float PRateRoll, IRateRoll, DRateRoll;

// Saídas para motores
extern float MotorInput1, MotorInput2;

// Analisadores para Ziegler-Nichols
extern KuPuAnalyzer rateAnalysis;
extern KuPuAnalyzer angleAnalysis;

// ============================================================================
// CONFIGURAÇÕES DE DEBUG (OPCIONAIS)
// ============================================================================

// Descomente para ativar debug detalhado
// #define DEBUG_MPU6050
// #define DEBUG_KALMAN  
// #define DEBUG_PID
// #define DEBUG_MOTORES

#endif