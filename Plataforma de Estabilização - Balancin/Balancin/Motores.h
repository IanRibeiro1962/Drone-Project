// Motores.h - Controle dos Motores e ESCs (VERSÃO CORRIGIDA)
#ifndef MOTORES_H
#define MOTORES_H

#include "Config.h"

uint8_t canalPWM1 = 0;
uint8_t canalPWM2 = 1;
uint32_t frequencia = 50;
uint8_t resolucao = 16;

bool motoresConfigurados = false;
bool sistemaArmado = false;

bool setupMotores() {
  // Verifica se os pinos são válidos
  for (int i = 0; i < numChannels; i++) {
    if (motorsPins[i] < 0) {
      Serial.println("ERRO: Pino de motor inválido");
      return false;
    }
  }
  
  // Configura canais PWM
  if (!ledcAttachChannel(motorsPins[0], frequencia, resolucao, canalPWM1)) {
    Serial.println("ERRO: Falha configurando canal PWM 1");
    return false;
  }
  
  if (!ledcAttachChannel(motorsPins[1], frequencia, resolucao, canalPWM2)) {
    Serial.println("ERRO: Falha configurando canal PWM 2");
    return false;
  }
  
  // Inicializa com pulso mínimo (motores desarmados)
  ledcWrite(motorsPins[0], ESC_PULSE_MIN);
  ledcWrite(motorsPins[1], ESC_PULSE_MIN);
  
  motoresConfigurados = true;
  Serial.println("Motores configurados com sucesso!");
  return true;
}

bool calibrateESCs() {
  if (!motoresConfigurados) {
    Serial.println("ERRO: Motores não configurados");
    return false;
  }
  
  Serial.println("=== INICIANDO CALIBRAÇÃO DOS ESCs ===");
  
  unsigned long timeout = millis();
  
  // 1) Pulso máximo
  Serial.println("-> Enviando pulso MÁXIMO (2 ms)");
  for (int i = 0; i < numChannels; i++) {
    ledcWrite(motorsPins[i], ESC_PULSE_MAX);
  }
  
  // Verifica timeout durante espera
  timeout = millis();
  while (millis() - timeout < CALIB_DELAY_HIGH) {
    delay(100);
    if (millis() - timeout > 10000) { // Timeout de 10 segundos
      Serial.println("ERRO: Timeout na calibração");
      return false;
    }
  }
  
  // 2) Pulso mínimo
  Serial.println("-> Enviando pulso MÍNIMO (1 ms)");
  for (int i = 0; i < numChannels; i++) {
    ledcWrite(motorsPins[i], ESC_PULSE_MIN);
  }
  
  timeout = millis();
  while (millis() - timeout < CALIB_DELAY_LOW) {
    delay(100);
    if (millis() - timeout > 10000) {
      Serial.println("ERRO: Timeout na calibração");
      return false;
    }
  }
  
  Serial.println("=== CALIBRAÇÃO CONCLUÍDA COM SUCESSO ===");
  return true;
}

void writeMotores(float motor1, float motor2) {
  if (!motoresConfigurados) {
    Serial.println("AVISO: Tentativa de escrever em motores não configurados");
    return;
  }
  
  // Aplica limites de segurança
  motor1 = constrain(motor1, ESC_PULSE_MIN, ESC_PULSE_MAX);
  motor2 = constrain(motor2, ESC_PULSE_MIN, ESC_PULSE_MAX);
  
  ledcWrite(motorsPins[0], motor1);
  ledcWrite(motorsPins[1], motor2);
}

void armarMotores() {
  if (!motoresConfigurados) {
    Serial.println("ERRO: Motores não configurados para armar");
    return;
  }
  
  writeMotores(ESC_PULSE_MIN, ESC_PULSE_MIN);
  sistemaArmado = true;
  Serial.println("Motores ARMADOS");
}

void desarmarMotores() {
  writeMotores(ESC_PULSE_MIN, ESC_PULSE_MIN);
  sistemaArmado = false;
  Serial.println("Motores DESARMADOS");
}

bool motoresArmados() {
  return sistemaArmado;
}

#endif