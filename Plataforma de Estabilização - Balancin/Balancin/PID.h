// PID.h - Controlador PID em Cascata (VERSÃO SIMPLIFICADA)
#ifndef PID_H
#define PID_H

#include "Config.h"

// ==================================================
// ESTRUTURA PARA ESTADO DO PID
// ==================================================
typedef struct {
  float erroAnterior;
  float termoIntegralAnterior;
  float medicaoAnterior;
  float saidaAnterior;
} EstadoPID;

// Estados para os dois PIDs em cascata
EstadoPID estadoPIDAngulo = {0, 0, 0, 0};
EstadoPID estadoPIDTaxa = {0, 0, 0, 0};

// ==================================================
// FUNÇÃO PID SIMPLIFICADA
// ==================================================
float pid_equation(float setpoint, float medicao, float Kp, float Ki, float Kd, 
                   EstadoPID &estado, float dt, float limiteSaida) {
  
  float erro = setpoint - medicao;
  
  // Termo Proporcional
  float termoProporcional = Kp * erro;
  
  // Termo Integral
  float termoIntegral = estado.termoIntegralAnterior + Ki * erro * dt;
  termoIntegral = constrain(termoIntegral, -limiteSaida, limiteSaida);
  
  // Termo Derivativo
  float termoDerivativo = 0;
  if (dt > 0) {
    termoDerivativo = Kd * (erro - estado.erroAnterior) / dt;
  }
  
  // Saída PID
  float saidaPID = termoProporcional + termoIntegral + termoDerivativo;
  saidaPID = constrain(saidaPID, -limiteSaida, limiteSaida);
  
  // Atualiza estado
  estado.erroAnterior = erro;
  estado.termoIntegralAnterior = termoIntegral;
  estado.medicaoAnterior = medicao;
  estado.saidaAnterior = saidaPID;
  
  return saidaPID;
}

// ==================================================
// FUNÇÕES PARA CASCATA
// ==================================================

// PID do laço externo (Ângulo)
float pidAngulo(float setpointAngulo, float medicaoAngulo, float dt) {
  return pid_equation(setpointAngulo, medicaoAngulo, 
                     PAngleRoll, IAngleRoll, DAngleRoll,
                     estadoPIDAngulo, dt, MAX_ANGLE_OUTPUT);
}

// PID do laço interno (Taxa)
float pidTaxa(float setpointTaxa, float medicaoTaxa, float dt) {
  return pid_equation(setpointTaxa, medicaoTaxa,
                     PRateRoll, IRateRoll, DRateRoll,
                     estadoPIDTaxa, dt, MAX_RATE_OUTPUT);
}

// Controle em cascata completo
void controleCascataCompleto(float setpointAngulo, float medicaoAngulo, 
                            float medicaoTaxa, float dt,
                            float &saidaAngulo, float &saidaTaxa) {
  
  // Laço externo: Ângulo -> Taxa desejada
  saidaAngulo = pidAngulo(setpointAngulo, medicaoAngulo, dt);
  saidaAngulo = constrain(saidaAngulo, -MAX_ANGLE_OUTPUT, MAX_ANGLE_OUTPUT);
  
  // Laço interno: Taxa desejada -> Correção do motor
  saidaTaxa = pidTaxa(saidaAngulo, medicaoTaxa, dt);
  saidaTaxa = constrain(saidaTaxa, -MAX_RATE_OUTPUT, MAX_RATE_OUTPUT);
}

// ==================================================
// FUNÇÕES DE CONFIGURAÇÃO
// ==================================================

void configurarPIDAngulo(float Kp, float Ki, float Kd) {
  PAngleRoll = Kp;
  IAngleRoll = Ki;
  DAngleRoll = Kd;
}

void configurarPIDTaxa(float Kp, float Ki, float Kd) {
  PRateRoll = Kp;
  IRateRoll = Ki;
  DRateRoll = Kd;
}

void aplicarZieglerNicholsAngulo(float Ku, float Pu) {
  if (Pu > 0 && Ku > 0) {
    PAngleRoll = 0.6 * Ku;
    IAngleRoll = (1.2 * Ku) / Pu;
    DAngleRoll = 0.075 * Ku * Pu;
  }
}

void aplicarZieglerNicholsTaxa(float Ku, float Pu) {
  if (Pu > 0 && Ku > 0) {
    PRateRoll = 0.6 * Ku;
    IRateRoll = (1.2 * Ku) / Pu;
    DRateRoll = 0.075 * Ku * Pu;
  }
}

void reinicializarPID() {
  estadoPIDAngulo = {0, 0, 0, 0};
  estadoPIDTaxa = {0, 0, 0, 0};
  Serial.println("PID reinicializado!");
}

#endif