#include <Arduino.h>

const int escPin = 19;    // Escolha um pino PWM adequado no ESP32
uint32_t freq = 50;       // Frequência de 50 Hz para o ESC
uint8_t pwmChannel = 0;   //Canal do Pwm do esp32
uint8_t resolution = 16;  // Resolução de 16 bits

//Usando uma resolução de 16 bits (valores de 0 a 65535), 1 ms (Vel Minima) corresponderá a aproximadamente 3276 e 2 ms a 6553 (Vel Máxima).


void setup() {
  // Configura o canal LEDC com a frequência e resolução definidas

  ledcAttachChannel(escPin, freq, resolution,pwmChannel);

  // Envia um pulso mínimo para inicializar o ESC (geralmente 1 ms, ou seja, ~3276)
  ledcWrite(escPin, 3276);

  // Aguarda alguns segundos para que o ESC possa calibrar ou armar
  delay(2000);
}

void loop() {
  // Exemplo: variação da velocidade de forma gradual
  for (int pulse = 3276; pulse <= 6553; pulse += 100) {
    ledcWrite(escPin, pulse);
    delay(100);
  }
  for (int pulse = 6553; pulse >= 3276; pulse -= 100) {
    ledcWrite(escPin, pulse);
    delay(100);
  }
}
