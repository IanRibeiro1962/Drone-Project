// Analisadores.h - Analisadores para Ziegler-Nichols (VERSÃO MELHORADA)
#ifndef ANALISADORES_H
#define ANALISADORES_H

#include "Estruturas.h"

// ============================================================================
// CONFIGURAÇÃO DO ANALISADOR
// ============================================================================

void initAnalyzer(KuPuAnalyzer* analyzer, float threshold, uint8_t minPeaks) {
  /**
   * INICIALIZA O ANALISADOR PARA ZIEGLER-NICHOLS
   * --------------------------------------------
   * threshold: Limite para detecção de picos (ex: 5.0 para ângulo, 50.0 para taxa)
   * minPeaks: Número mínimo de picos para cálculo confiável (recomendado: 3-5)
   */
  analyzer->threshold = threshold;
  analyzer->minPeaks = minPeaks;
  analyzer->lastValue = 0;
  analyzer->lastPeakTime = 0;
  analyzer->peakCount = 0;
  analyzer->periodSum = 0;
  analyzer->Ku = 0;
  analyzer->Pu = 0;

  Serial.print(" Analisador configurado - Threshold: ");
  Serial.print(threshold);
  Serial.print(" | Mín. picos: ");
  Serial.println(minPeaks);
}

// ============================================================================
// ATUALIZAÇÃO DA ANÁLISE COM PROTEÇÕES
// ============================================================================

void updateAnalysis(KuPuAnalyzer* analyzer, float currentValue, float currentGain) {
  /**
   * ATUALIZA A ANÁLISE COM NOVO VALOR - DETECTA OSCILAÇÕES SUSTENTADAS
   * -------------------------------------------------------------------
   * currentValue: Valor atual do sistema (ângulo ou taxa)
   * currentGain: Ganho P atual sendo testado
   * 
   * PROTEÇÕES IMPLEMENTADAS:
   * 1. Detecção de borda de subida
   * 2. Filtro de tempo mínimo entre picos
   * 3. Validação de parâmetros obtidos
   * 4. Reset automático em caso de valores inválidos
   */

  // Detecção de borda de subida (cruzamento do threshold para cima)
  if ((currentValue > analyzer->threshold) && (analyzer->lastValue <= analyzer->threshold)) {
    unsigned long now = micros();

    // PROTEÇÃO 1: Ignora primeiro pico (precisa de referência temporal)
    if (analyzer->lastPeakTime != 0) {

      // PROTEÇÃO 2: Filtro de tempo mínimo entre picos (evita ruído)
      unsigned long timeSinceLastPeak = now - analyzer->lastPeakTime;
      float minPeriod = 0.01f;  // 10ms mínimo entre picos (100Hz max)

      if (timeSinceLastPeak >= minPeriod * 1000000) {  // Converte para microssegundos

        // Calcula período entre picos (em segundos)
        float period = timeSinceLastPeak / 1000000.0f;
        analyzer->periodSum += period;
        analyzer->peakCount++;

        // Debug opcional
        Serial.print(" Pico detectado! #");
        Serial.print(analyzer->peakCount);
        Serial.print(" | Período: ");
        Serial.print(period, 4);
        Serial.println("s");

        // PROTEÇÃO 3: Calcula Ku e Pu apenas com número suficiente de picos
        if (analyzer->peakCount >= analyzer->minPeaks) {
          analyzer->Pu = analyzer->periodSum / (analyzer->peakCount - 1);  // Média dos períodos
          analyzer->Ku = currentGain;

          // PROTEÇÃO 4: Validação rigorosa dos parâmetros obtidos
          bool puValid = (analyzer->Pu > 0.05f) && (analyzer->Pu < 5.0f);    // Período entre 50ms e 5s
          bool kuValid = (analyzer->Ku > 0.01f) && (analyzer->Ku < 100.0f);  // Ganho entre 0.01 e 100

          if (puValid && kuValid) {
            Serial.println(" =========================================");
            Serial.println(" PARÂMETROS CRÍTICOS DETECTADOS!");
            Serial.print(" Ku (Ganho Crítico): ");
            Serial.println(analyzer->Ku, 4);
            Serial.print(" Pu (Período Crítico): ");
            Serial.print(analyzer->Pu, 4);
            Serial.println(" segundos");
            Serial.println(" =========================================");

            // Aplica Ziegler-Nichols automaticamente (OPCIONAL)
            // aplicarZieglerNichols(analyzer->Ku, analyzer->Pu);

          } else {
            Serial.println(" Parâmetros inválidos detectados - reiniciando análise");
            Serial.print("   Pu: ");
            Serial.print(analyzer->Pu, 4);
            Serial.print(" | Ku: ");
            Serial.println(analyzer->Ku, 4);

            // PROTEÇÃO 5: Reset automático em caso de valores inválidos
            analyzer->periodSum = 0;
            analyzer->peakCount = 0;
            analyzer->lastPeakTime = now;  // Reset com tempo atual
            return;
          }

          // Reinicia para próxima análise (mantém Ku e Pu)
          analyzer->periodSum = 0;
          analyzer->peakCount = 0;
        }
      } else {
        Serial.println("⏱ Pico ignorado - muito próximo do anterior (ruído?)");
      }
    }

    // Atualiza tempo do último pico (mesmo que seja o primeiro)
    analyzer->lastPeakTime = now;
  }

  // Atualiza último valor para próxima detecção de borda
  analyzer->lastValue = currentValue;
}

// ============================================================================
// FUNÇÕES AUXILIARES
// ============================================================================

void resetAnalyzer(KuPuAnalyzer* analyzer) {
  /**
   * REINICIA O ANALISADOR MANTENDO CONFIGURAÇÃO
   * -------------------------------------------
   * Útil para iniciar nova análise sem perder threshold e minPeaks
   */
  analyzer->lastValue = 0;
  analyzer->lastPeakTime = 0;
  analyzer->peakCount = 0;
  analyzer->periodSum = 0;
  // Mantém Ku e Pu dos últimos resultados válidos
  Serial.println(" Analisador reiniciado - pronto para nova análise");
}

float getKu(const KuPuAnalyzer* analyzer) {
  return analyzer->Ku;
}

float getPu(const KuPuAnalyzer* analyzer) {
  return analyzer->Pu;
}

bool analysisComplete(const KuPuAnalyzer* analyzer) {
  return (analyzer->Ku > 0) && (analyzer->Pu > 0);
}

#endif