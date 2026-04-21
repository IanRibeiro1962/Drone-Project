// Debug.h - Sistema de Debug Resumido
#ifndef DEBUG_H
#define DEBUG_H

#include "Config.h"
#include "Estruturas.h"

// ==================================================
// CONTROLE DE DEBUG POR COMPONENTE
// ==================================================
#define DEBUG_GIROSCOPIO     0    // Teste do giroscópio (1 linha)
#define DEBUG_KALMAN         0   // Teste do filtro de Kalman  
#define DEBUG_MOTORES        0    // Teste dos motores
#define DEBUG_PID            0    // Teste do PID
#define DEBUG_ANALISADORES   1    // Teste dos analisadores Z-N
#define DEBUG_SISTEMA        0    // Debug geral do sistema

// ==================================================
// DEBUG RESUMIDO - 1 LINHA POR COMPONENTE
// ==================================================

void debugGiroscopioResumido() {
  #if DEBUG_GIROSCOPIO
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 100) { // 10Hz
    // Formato: GYRO_R:roll°s P:pitch°s Y:yaw°s | ACC_R:roll° P:pitch°
    Serial.print("GYRO_R:");
    Serial.print(RateRoll, 1);
    Serial.print("°s P:");
    Serial.print(RatePitch, 1);
    Serial.print("°s Y:");
    Serial.print(RateYaw, 1);
    Serial.print("°s | ACC_R:");
    Serial.print(AngleRoll, 1);
    Serial.print("° P:");
    Serial.print(AnglePitch, 1);
    Serial.println("°");
    
    lastDebug = millis();
  }
  #endif
}

void debugKalmanResumido() {
  #if DEBUG_KALMAN
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 100) {
    // Formato: KAL_R:roll° U:incerteza | DIF:diferença°
    Serial.print("KAL_R:");
    Serial.print(KalmanAngleRoll, 2);
    Serial.print("° U:");
    Serial.print(KalmanUncertaintyAngleRoll, 4);
    Serial.print(" | DIF:");
    Serial.print(KalmanAngleRoll - AngleRoll, 2);
    Serial.println("°");
    
    lastDebug = millis();
  }
  #endif
}

void debugMotoresResumido() {
  #if DEBUG_MOTORES
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 200) {
    // Formato: MOT_M1:pwm(p%) M2:pwm(p%) | ARM:sim/não
    float m1_percent = ((MotorInput1 - ESC_PULSE_MIN) * 100.0) / (ESC_PULSE_MAX - ESC_PULSE_MIN);
    float m2_percent = ((MotorInput2 - ESC_PULSE_MIN) * 100.0) / (ESC_PULSE_MAX - ESC_PULSE_MIN);
    
    Serial.print("MOT_M1:");
    Serial.print(MotorInput1);
    Serial.print("(");
    Serial.print(m1_percent, 0);
    Serial.print("%) M2:");
    Serial.print(MotorInput2);
    Serial.print("(");
    Serial.print(m2_percent, 0);
    Serial.print("%) | ARM:");
    Serial.println(motoresArmados() ? "SIM" : "NÃO");
    
    lastDebug = millis();
  }
  #endif
}

void debugPIDResumido() {
  #if DEBUG_PID
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 200) {
    // Formato: PID_ERR:erro° OUT:saída | P:p I:i D:d
    Serial.print("PID_ERR:");
    Serial.print(ErrorAngleRoll, 2);
    Serial.print("° OUT:");
    Serial.print(InputRoll, 1);
    Serial.print(" | P:");
    Serial.print(PAngleRoll, 3);
    Serial.print(" I:");
    Serial.print(IAngleRoll, 3);
    Serial.print(" D:");
    Serial.print(DAngleRoll, 3);
    Serial.println();
    
    lastDebug = millis();
  }
  #endif
}

void debugSistemaResumido() {
  #if DEBUG_SISTEMA
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 200) {
    // Formato: SIS_R:roll° E:erro° C:correção A:armado
    Serial.print("SIS_R:");
    Serial.print(KalmanAngleRoll, 1);
    Serial.print("° E:");
    Serial.print(ErrorAngleRoll, 1);
    Serial.print("° C:");
    Serial.print(InputRoll, 0);
    Serial.print(" A:");
    Serial.println(motoresArmados() ? "SIM" : "NÃO");
    
    lastDebug = millis();
  }
  #endif
}

// ==================================================
// FUNÇÕES DE TESTE ESPECÍFICAS (MANTIDAS)
// ==================================================

void testeApenasGiroscopio() {
  if (readMPU6050()) {
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
    
    debugGiroscopioResumido();
  }
}

void testeKalman() {
  if (readMPU6050()) {
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
    
    FiltroKalman();
    
    debugKalmanResumido();
  }
}

void testeMotoresSequencia() {
  static unsigned long startTime = millis();
  static uint16_t testePWM = ESC_PULSE_MIN;
  static uint8_t fase = 0;
  
  unsigned long tempoDecorrido = millis() - startTime;
  
  if (tempoDecorrido < 2000) {
    if (fase != 1) {
      Serial.println("🔧 FASE 1: Motores em 0%");
      fase = 1;
    }
    testePWM = ESC_PULSE_MIN;
  }
  else if (tempoDecorrido < 4000) {
    if (fase != 2) {
      Serial.println("🔧 FASE 2: Motores em 10%");
      fase = 2;
    }
    testePWM = ESC_PULSE_MIN + (ESC_PULSE_MAX - ESC_PULSE_MIN) * 0.1;
  }
  else if (tempoDecorrido < 6000) {
    if (fase != 3) {
      Serial.println("🔧 FASE 3: Motores em 0%");
      fase = 3;
    }
    testePWM = ESC_PULSE_MIN;
  }
  else if (tempoDecorrido < 8000) {
    if (fase != 4) {
      Serial.println("🔧 FASE 4: Motores em 20%");
      fase = 4;
    }
    testePWM = ESC_PULSE_MIN + (ESC_PULSE_MAX - ESC_PULSE_MIN) * 0.2;
  }
  else {
    startTime = millis();
    fase = 0;
    return;
  }
  
  writeMotores(testePWM, testePWM);
  debugMotoresResumido();
}

// ==================================================
// GERENCIADOR PRINCIPAL DE DEBUG
// ==================================================

void gerenciarDebug() {
  // Apenas chama as funções resumidas - cada uma controla seu timing
  debugGiroscopioResumido();
  debugKalmanResumido();
  debugMotoresResumido();
  debugPIDResumido();
  debugSistemaResumido();
}

void debugInicializacao() {
  Serial.println("\n==================================================");
  Serial.println("           SISTEMA BALANCIN - DEBUG RESUMIDO");
  Serial.println("==================================================");
  Serial.println("LEGENDA:");
  Serial.println("GYRO_R: Roll gyro °/s, P: Pitch gyro °/s, Y: Yaw gyro °/s");
  Serial.println("ACC_R: Roll accelerometer °, P: Pitch accelerometer °");
  Serial.println("KAL_R: Roll Kalman °, U: Incerteza");
  Serial.println("MOT_M1: Motor1 PWM (%), M2: Motor2 PWM (%)");
  Serial.println("PID_ERR: Erro ângulo °, OUT: Saída correção");
  Serial.println("SIS_R: Roll sistema °, E: Erro °, C: Correção, A: Armado");
  Serial.println("==================================================");
  
  // Status dos debugs ativos
  Serial.print("DEBUGS ATIVOS: ");
  #if DEBUG_GIROSCOPIO
  Serial.print("GIRO ");
  #endif
  #if DEBUG_KALMAN
  Serial.print("KALMAN ");
  #endif
  #if DEBUG_MOTORES
  Serial.print("MOTORES ");
  #endif
  #if DEBUG_PID
  Serial.print("PID ");
  #endif
  #if DEBUG_SISTEMA
  Serial.print("SISTEMA ");
  #endif
  Serial.println();
  Serial.println("==================================================");
}

// ==================================================
// FUNÇÕES AUXILIARES
// ==================================================

void debugMensagem(String modulo, String mensagem) {
  Serial.print("[");
  Serial.print(modulo);
  Serial.print("] ");
  Serial.println(mensagem);
}

#endif