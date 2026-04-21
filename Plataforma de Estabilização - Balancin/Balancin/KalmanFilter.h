// KalmanFilter.h - Implementação do Filtro de Kalman (VERSÃO PRONTA PARA PRODUÇÃO)
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "Config.h"

// PARÂMETROS DO FILTRO DE KALMAN
// ==============================
float Q_angle = 0.002;   // Valor padrão - SERÁ SUBSTITUÍDO na calibração
float R_angle = 0.8;     // Valor padrão - SERÁ SUBSTITUÍDO na calibração

void inicializarKalman() {
  /**
   * INICIALIZAÇÃO INTELIGENTE DO FILTRO
   * -----------------------------------
   * Usa os ângulos atuais do acelerômetro como ponto de partida
   * Isso garante uma convergência rápida do filtro
   */
  KalmanAngleRoll = AngleRoll;        
  KalmanAnglePitch = AnglePitch;        
  KalmanUncertaintyAngleRoll = R_angle;   
  KalmanUncertaintyAnglePitch = R_angle;

  Serial.println("✅ Filtro de Kalman inicializado com ângulos atuais");
}

void FiltroKalman() {
  /**
   * EXECUÇÃO DO FILTRO DE KALMAN - CHAMAR A CADA CICLO DE CONTROLE
   * --------------------------------------------------------------
   * Combina giroscópio (rápido) com acelerômetro (preciso)
   * Ts deve ser o tempo real entre iterações (calculado no loop principal)
   */
  
  // Verificação de segurança
  if (isnan(RateRoll) || isnan(AngleRoll)) {
    Serial.println("⚠️ AVISO: Dados inválidos - pulando filtro Kalman");
    return;
  }

  // ========== EIXO ROLL (PRINCIPAL) ==========
  KalmanAngleRoll = KalmanAngleRoll + Ts * RateRoll;
  KalmanUncertaintyAngleRoll = KalmanUncertaintyAngleRoll + Ts * Ts * Q_angle;

  float KalmanGainRoll = KalmanUncertaintyAngleRoll / (KalmanUncertaintyAngleRoll + R_angle);
  KalmanAngleRoll = KalmanAngleRoll + KalmanGainRoll * (AngleRoll - KalmanAngleRoll);
  KalmanUncertaintyAngleRoll = (1 - KalmanGainRoll) * KalmanUncertaintyAngleRoll;

  // ========== EIXO PITCH (SECUNDÁRIO) ==========
  KalmanAnglePitch = KalmanAnglePitch + Ts * RatePitch;
  KalmanUncertaintyAnglePitch = KalmanUncertaintyAnglePitch + Ts * Ts * Q_angle;

  float KalmanGainPitch = KalmanUncertaintyAnglePitch / (KalmanUncertaintyAnglePitch + R_angle);
  KalmanAnglePitch = KalmanAnglePitch + KalmanGainPitch * (AnglePitch - KalmanAnglePitch);
  KalmanUncertaintyAnglePitch = (1 - KalmanGainPitch) * KalmanUncertaintyAnglePitch;

  // Saídas para debug
  Kalman1DOutput[0] = KalmanAngleRoll;
  Kalman1DOutput[1] = KalmanUncertaintyAngleRoll;
}

// ============================================================================
// CALIBRAÇÃO COMPLETA - USAR APENAS UMA VEZ PARA OBTER VALORES OTIMIZADOS
// ============================================================================

bool collectAllanData() {
  /**
   * COLETA DADOS PARA ALLAN VARIANCE - MANTENHA O SENSOR IMÓVEL!
   */
  Serial.println("📊 COLETANDO DADOS PARA ALLAN VARIANCE");
  Serial.println("❕ MANTENHA O SENSOR COMPLETAMENTE IMÓVEL!");

  unsigned long startTime = millis();
  int successfulReads = 0;

  for (int i = 0; i < sampleCount; i++) {
    if (readMPU6050()) {
      AccDataXZ[i] = RateRoll;
      AccDataYZ[i] = RatePitch;
      successfulReads++;
    }

    // Feedback progressivo
    if (i % (sampleCount / 10) == 0) {
      Serial.print("📈 Progresso: ");
      Serial.print((i * 100) / sampleCount);
      Serial.println("%");
    }
    delay(1);
  }

  Serial.print("✅ Coleta concluída: ");
  Serial.print(successfulReads);
  Serial.print("/");
  Serial.print(sampleCount);
  Serial.println(" amostras");
  
  return (successfulReads == sampleCount);
}

float computeAllanVariance(float* data, int n, float Ts, int clusterSize) {
  if (n < clusterSize * 2) return 0;
  
  int M = n / clusterSize;
  if (M < 2) return 0;

  float sum = 0;
  int validPairs = 0;

  for (int i = 0; i < M - 1; i++) {
    float avg1 = 0, avg2 = 0;

    for (int j = 0; j < clusterSize; j++) {
      int index1 = i * clusterSize + j;
      int index2 = (i + 1) * clusterSize + j;

      if (index1 < n && index2 < n) {
        avg1 += data[index1];
        avg2 += data[index2];
      }
    }

    avg1 /= clusterSize;
    avg2 /= clusterSize;

    if (!isnan(avg1) && !isnan(avg2)) {
      sum += (avg2 - avg1) * (avg2 - avg1);
      validPairs++;
    }
  }

  if (validPairs < 1) return 0;

  float tau = clusterSize * Ts;
  return (sum / (2.0 * validPairs)) / (tau * tau);
}

void calibrarKalman() {
  /**
   * CALIBRAÇÃO COMPLETA - EXECUTAR APENAS UMA VEZ!
   * -------------------------------------------------
   * ESTA FUNÇÃO DEVE SER USADA APENAS NA PRIMEIRA CONFIGURAÇÃO
   * OU QUANDO MUDAR O SENSOR/AMBIENTE
   * 
   * INSTRUÇÕES:
   * 1. Execute esta função UMA VEZ com o sensor imóvel
   * 2. Anote os valores de Q e R que aparecem no Serial
   * 3. Substitua os valores em calibrarKalmanRapida()
   * 4. Use calibrarKalmanRapida() daqui para frente
   */
  
  Serial.println("===========================================");
  Serial.println(" CALIBRAÇÃO COMPLETA DO FILTRO KALMAN");
  Serial.println(" (EXECUTAR APENAS UMA VEZ!)");
  Serial.println("===========================================");

  // Coleta de dados
  if (!collectAllanData()) {
    Serial.println("Falha na coleta - usando valores padrão");
    return;
  }

  // Cálculo da Allan Variance
  float allanVar1 = computeAllanVariance(AccDataXZ, sampleCount, Ts_calib, 10);
  float allanVar2 = computeAllanVariance(AccDataXZ, sampleCount, Ts_calib, 50);
  float allanVar3 = computeAllanVariance(AccDataXZ, sampleCount, Ts_calib, 100);

  Q_angle = (allanVar1 + allanVar2 + allanVar3) / 3.0;

  // Estima ruído do acelerômetro
  float sumVar = 0;
  for (int i = 0; i < sampleCount; i++) {
    if (readMPU6050()) {
      sumVar += AngleRoll * AngleRoll;
    }
    delay(1);
  }
  R_angle = sumVar / sampleCount;

  Serial.println("🔧 ===========================================");
  Serial.println("🔧 VALORES OBTIDOS - ANOTE-OS!");
  Serial.print("🔧 Q_angle (ruído giroscópio) = ");
  Serial.println(Q_angle, 6);
  Serial.print("🔧 R_angle (ruído acelerômetro) = ");
  Serial.println(R_angle, 6);
  Serial.println("🔧 ===========================================");
  Serial.println("🔧 INSTRUÇÃO: Substitua estes valores na");
  Serial.println("🔧 função calibrarKalmanRapida() e use-a");
  Serial.println("🔧 daqui para frente!");
  Serial.println("🔧 ===========================================");
}

// ============================================================================
// CALIBRAÇÃO RÁPIDA - USAR SEMPRE APÓS A PRIMEIRA CALIBRAÇÃO
// ============================================================================

void calibrarKalmanRapida() {
  /**
   * CALIBRAÇÃO RÁPIDA - USAR SEMPRE APÓS A PRIMEIRA CALIBRAÇÃO
   * ------------------------------------------------------------
   * INSTRUÇÕES IMPORTANTES:
   * 
   * 1. PRIMEIRO: Execute calibrarKalman() UMA VEZ e anote os valores
   * 2. SUBSTITUA: Os valores abaixo pelos que você obteve
   * 3. USE: Esta função em todos os projetos futuros
   * 
   * VALORES ATUAIS: São exemplos - SUBSTITUA pelos seus valores!
   */
  
  Serial.println("===========================================");
  Serial.println("CALIBRAÇÃO RÁPIDA - VALORES OTIMIZADOS");
  Serial.println("===========================================");

  // ════════════════════════════════════════════════════════════
  //                  SUBSTITUIR ESTES VALORES!  
  // ════════════════════════════════════════════════════════════
  
  // VALORES OBTIDOS DA CALIBRAÇÃO COMPLETA - SUBSTITUIR!
  Q_angle = 0.0012;    // ← SUBSTITUIR pelo seu valor de Q
  R_angle = 0.65;      // ← SUBSTITUIR pelo seu valor de R
  
  // ════════════════════════════════════════════════════════════

  Serial.print("Q_angle = ");
  Serial.println(Q_angle, 6);
  Serial.print("R_angle = ");
  Serial.println(R_angle, 6);
  Serial.println("Filtro Kalman calibrado e pronto!");
  Serial.println("===========================================");
}

#endif