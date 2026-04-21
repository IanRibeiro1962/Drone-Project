// Balancin.ino - Arquivo Principal (VERSÃO COM COMPENSAÇÃO DE MOTORES)
#include "Config.h"
#include "Estruturas.h"
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "PID.h"
#include "Motores.h"
#include "Analisadores.h"
#include "Debug.h"

// DEFINIÇÕES REAIS das variáveis globais
float AccDataXZ[sampleCount];
float AccDataYZ[sampleCount];
float allanVarXZ, allanVarYZ;

float Ts, lastTime, currentTime, Ts_calib;
unsigned long lastControlTime;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// Variáveis de controle
float InputRoll, InputThrottle;
float DesiredAngleRoll, DesiredRateRoll;
float ErrorAngleRoll, ErrorRateRoll;

// Variáveis para o controle em cascata
float saidaAngulo, saidaTaxa;

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[2] = { 0, 0 };

// Definições dos analisadores
KuPuAnalyzer rateAnalysis, angleAnalysis;

// Parâmetros PID - VALORES INICIAIS SEGUROS
float PAngleRoll = 0.0, IAngleRoll = 0.0, DAngleRoll = 0.0;  // Ângulo DESLIGADO inicialmente
float PRateRoll = 43, IRateRoll = 0.0, DRateRoll = 0.0;      // Taxa com ganho baixo

// Variáveis para motores
float MotorInput1, MotorInput2;

// ============================================================================
// CONFIGURAÇÃO DE COMPENSAÇÃO DOS MOTORES
// ============================================================================

void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(1000);  // Delay maior para estabilização

  Serial.println("🎯 SISTEMA BALANCIN - INICIALIZANDO");
  Serial.println("⚠️  REMOVA AS HÉLICES PARA TESTES INICIAIS!");
  Serial.print("🔧 Throttle mínimo: ");
  Serial.print(THROTTLE_MINIMO);
  Serial.print(" (");
  Serial.print(((THROTTLE_MINIMO - ESC_PULSE_MIN) * 100.0) / (ESC_PULSE_MAX - ESC_PULSE_MIN), 1);
  Serial.println("%)");
  Serial.print("🔧 Compensação motores: ");
  Serial.println(COMPENSACAO_MOTOR);

  // ==========================================================================
  // CONFIGURAÇÃO INICIAL - FASE 1: PREPARAÇÃO
  // ==========================================================================

  // 1. Configuração dos PIDs - COMEÇE COM GANHOS BAIXOS!
  configurarPIDAngulo(PAngleRoll, IAngleRoll, DAngleRoll);
  configurarPIDTaxa(PRateRoll, IRateRoll, DRateRoll);

  // 2. Inicialização dos sistemas
  if (!setupMotores()) {
    Serial.println("❌ FALHA NA CONFIGURAÇÃO DOS MOTORES - PARANDO");
    while (1)
      ;  // Trava o sistema
  }

  Serial.println("✅ Motores configurados - INICIANDO CALIBRAÇÃO DOS ESCs");
  if (!calibrateESCs()) {
    Serial.println("❌ FALHA NA CALIBRAÇÃO DOS ESCs - PARANDO");
    while (1)
      ;
  }

  // 3. Sensor MPU6050
  if (!setupMPU6050()) {
    Serial.println("❌ FALHA NO SENSOR MPU6050 - PARANDO");
    while (1)
      ;
  }

  Serial.println("✅ Sensor MPU6050 OK - INICIANDO CALIBRAÇÃO DOS COMPONENTES");
  componentsCalibrate();

  // 4. Filtro de Kalman - USE CALIBRAÇÃO RÁPIDA PARA TESTES
  Serial.println("✅ Componentes calibrados - CONFIGURANDO FILTRO KALMAN");
  calibrarKalmanRapida();  // ⚠️ Use calibrarKalman() apenas na primeira vez!
  inicializarKalman();

  // 5. Analisadores para Ziegler-Nichols
  initAnalyzer(&rateAnalysis, 15.0, 4);   // Threshold 30°/s para taxa
  initAnalyzer(&angleAnalysis, 8.0, 4);  // Threshold 10° para ângulo

  // 6. ARMAR MOTORES (APÓS TODAS AS CALIBRAÇÕES)
  Serial.println("🔓 ARMANDO MOTORES...");
  armarMotores();
  delay(2000);  // Aguarda 2 segundos para confirmação dos ESCs

  lastControlTime = micros();

  debugInicializacao();
  Serial.println("🎯 Sistema inicializado - PRONTO PARA CONTROLE");
  Serial.println("💡 INSTRUÇÃO: Ajuste gradualmente os ganhos do PID de taxa");
  Serial.println("💡 Comandos: '1'=Debug, '2'=Controle leve, '3'=Controle completo");
}

void loop() {
  currentTime = micros();

  // Controle em período fixo (100Hz = 10ms)
  if (currentTime - lastControlTime >= CONTROL_TS * 1000000) {
    Ts = (currentTime - lastControlTime) / 1000000.0;  // Ts REAL

    // ==========================================================================
    // LEITURA E PROCESSAMENTO DO SENSOR
    // ==========================================================================

    if (!readMPU6050()) {
      Serial.println("❌ Falha na leitura do MPU6050");
      lastControlTime = currentTime;
      return;
    }

    // Aplica calibração do giroscópio
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    // Filtro de Kalman para suavizar os ângulos
    FiltroKalman();

    // ==========================================================================
    // CONTROLE PID EM CASCATA
    // ==========================================================================

    DesiredAngleRoll = 0;  // Objetivo: manter balancin na horizontal (0°)
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;

    // 🎯 ESTRATÉGIA DE CONTROLE SEGURO:
    // Fase 1: Apenas controle de taxa (ângulo desligado)
    // Fase 2: Quando estável, ativar controle de ângulo

    if (abs(KalmanAngleRoll) < 5.0) {  // Só ativa ângulo se próximo do equilíbrio
      // Controle completo em cascata
      controleCascataCompleto(DesiredAngleRoll, KalmanAngleRoll, RateRoll, Ts, saidaAngulo, saidaTaxa);
    } else {
      // Apenas controle de taxa (mais seguro)
      saidaAngulo = 0;
      saidaTaxa = pidTaxa(0, RateRoll, Ts);  // Tenta reduzir taxa para zero
    }

    DesiredRateRoll = saidaAngulo;
    InputRoll = saidaTaxa;
    ErrorRateRoll = DesiredRateRoll - RateRoll;

    // ==========================================================================
    // SAÍDA PARA MOTORES COM COMPENSAÇÃO
    // ==========================================================================

    // THROTTLE MÍNIMO BASEADO NO SEU TESTE (7%)
    InputThrottle = THROTTLE_MINIMO;

    // 🔧 MIXAGEM DOS MOTORES COM COMPENSAÇÃO
    // A compensação ajuda a equalizar motores com respostas ligeiramente diferentes
    MotorInput1 = constrain(InputThrottle + InputRoll + COMPENSACAO_MOTOR, ESC_PULSE_MIN, ESC_PULSE_MAX);
    MotorInput2 = constrain(InputThrottle - InputRoll - COMPENSACAO_MOTOR, ESC_PULSE_MIN, ESC_PULSE_MAX);

    // ⚠️ VERIFICAÇÃO DE SEGURANÇA CRÍTICA
    if (abs(KalmanAngleRoll) > 25.0) {  // Ângulo máximo de segurança
      Serial.println("🚨 ÂNGULO CRÍTICO - DESARMANDO MOTORES!");
      desarmarMotores();
      while (1) {  // Trava o sistema
        Serial.println("⛔ SISTEMA TRAVADO POR SEGURANÇA - REINICIE MANUALMENTE");
        delay(5000);
      }
    }

    // Aplica apenas se motores armados
    if (motoresArmados()) {
      writeMotores(MotorInput1, MotorInput2);
    }

    // ==========================================================================
    // ANÁLISE E DEBUG
    // ==========================================================================

    // Atualiza análises para Ziegler-Nichols
    updateAnalysis(&rateAnalysis, RateRoll, PRateRoll);
    updateAnalysis(&angleAnalysis, KalmanAngleRoll, PAngleRoll);

    // Sistema de debug (não sobrecarregar o Serial)
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime > 100000) {  // 100ms
      gerenciarDebug();
      lastDebugTime = currentTime;
    }

    lastControlTime = currentTime;
  }

  // ============================================================================
  // CONTROLE POR TECLADO (OPCIONAL)
  // ============================================================================
  if (Serial.available()) {
    char comando = Serial.read();
    switch (comando) {
      case '1':  // Modo debug (sem controle)
        configurarPIDAngulo(0.0, 0.0, 0.0);
        configurarPIDTaxa(0.0, 0.0, 0.0);
        Serial.println("📊 Modo DEBUG - Controle desativado");
        break;
      case '2':  // Modo controle leve (apenas taxa)
        configurarPIDAngulo(0.0, 0.0, 0.0);
        configurarPIDTaxa(0.3, 0.0, 0.0);
        Serial.println("🎯 Modo CONTROLE LEVE - Apenas taxa");
        break;
      case '3':  // Modo controle completo
        configurarPIDAngulo(2.0, 0.0, 0.1);
        configurarPIDTaxa(0.3, 0.0, 0.02);
        Serial.println("🚀 Modo CONTROLE COMPLETO - Cascata ângulo+taxa");
        break;
      case 't':  // Teste de motores
        testeMotoresCompensado();
        break;
    }
  }

  lastTime = currentTime;
}

// ============================================================================
// FUNÇÕES AUXILIARES PARA TESTES
// ============================================================================

void testeMotoresCompensado() {
  /**
   * TESTE SEGURO DOS MOTORES COM COMPENSAÇÃO
   * Verifica se a compensação está funcionando
   */
  Serial.println("🔧 INICIANDO TESTE DE MOTORES COM COMPENSAÇÃO");

  if (!motoresArmados()) {
    armarMotores();
    delay(1000);
  }

  Serial.println("🎯 Teste 1: Motores sem compensação (teoricamente iguais)");
  writeMotores(THROTTLE_MINIMO, THROTTLE_MINIMO);
  delay(3000);

  Serial.println("🎯 Teste 2: Motores COM compensação");
  writeMotores(THROTTLE_MINIMO - COMPENSACAO_MOTOR, THROTTLE_MINIMO + COMPENSACAO_MOTOR);
  delay(3000);

  Serial.println("🎯 Teste 3: Motores em oposição (verificar mixagem)");
  writeMotores(THROTTLE_MINIMO + 50, THROTTLE_MINIMO - 50);
  delay(3000);

  // Volta ao mínimo
  writeMotores(ESC_PULSE_MIN, ESC_PULSE_MIN);
  Serial.println("✅ Teste de motores concluído");
}

void testeRespostaIndividual() {
  /**
   * TESTE INDIVIDUAL DE CADA MOTOR
   * Útil para verificar a compensação
   */
  Serial.println("🔧 TESTE INDIVIDUAL DE MOTORES");

  if (!motoresArmados()) {
    armarMotores();
    delay(1000);
  }

  Serial.println("⚙️  Motor 1 apenas");
  writeMotores(THROTTLE_MINIMO, ESC_PULSE_MIN);
  delay(2000);

  Serial.println("⚙️  Motor 2 apenas");
  writeMotores(ESC_PULSE_MIN, THROTTLE_MINIMO);
  delay(2000);

  Serial.println("⚙️  Ambos motores com compensação");
  writeMotores(THROTTLE_MINIMO - COMPENSACAO_MOTOR, THROTTLE_MINIMO + COMPENSACAO_MOTOR);
  delay(2000);

  // Parar
  writeMotores(ESC_PULSE_MIN, ESC_PULSE_MIN);
  Serial.println("✅ Teste individual concluído");
}