// Estruturas.h - Definições de Estruturas de Dados
#ifndef ESTRUTURAS_H
#define ESTRUTURAS_H

// Estrutura para análise Ziegler-Nichols
typedef struct {
  // Configuração
  float threshold;      // Limite para detecção de picos
  uint8_t minPeaks;     // Mínimo de picos para cálculo confiável
  
  // Estado interno
  float lastValue;      // Último valor para detecção de borda
  unsigned long lastPeakTime; // Tempo do último pico (microssegundos)
  uint16_t peakCount;   // Contador de picos detectados
  float periodSum;      // Soma acumulada dos períodos
  
  // Resultados
  float Ku;             // Ganho crítico (Ultimate Gain)
  float Pu;             // Período crítico (Ultimate Period)
} KuPuAnalyzer;

#endif