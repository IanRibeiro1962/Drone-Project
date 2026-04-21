// Teste ESC + motor brushless - Arduino Uno
// Uso: Serial monitor 115200
// Comandos via Serial:
//   'c' => calibrar (procedimento de calibração típico: MAX -> MIN)
//   'r' => rampa automática (0% -> 100% -> 0%)
//   's' => parar (envia MIN)
//   digits (0..100) => define throttle em %

#include <ESP32Servo.h>

Servo esc;

const int escPin = 13;       // pino de sinal para o ESC
const int MIN_PULSE = 1000;  // microsegundos (valor mínimo; ajuste se necessário)
const int MAX_PULSE = 2000;  // microsegundos (valor máximo; ajuste se necessário)

const int RAMP_STEP_MS = 50;  // tempo entre passos na rampa (ms)
const int RAMP_STEP_US = 10;  // incremento de microsegundos por passo (us)

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }  // aguarda porta serial (não trava se USB ausente)
  Serial.println();
  Serial.println("=== Teste ESC + BLDC ===");
  Serial.println("Conecte GND comum entre Arduino e ESC.");
  Serial.println("Remova a helice! Use sem helice para testes.");
  Serial.println("Comandos: 'c' calibrar, 'r' rampa, 's' parar, '0'..'100' throttle %");
  esc.attach(escPin, MIN_PULSE, MAX_PULSE);  // configura pulso mínimo/máximo
  delay(500);
  sendThrottlePercent(0);  // envia mínimo ao iniciar
  Serial.println("Pronto.");
}

void loop() {
  if (Serial.available()) {
    String t = Serial.readStringUntil('\n');
    t.trim();
    if (t.length() == 0) return;
    char cmd = t.charAt(0);
    if (cmd == 'c' || cmd == 'C') {
      calibrateESC();
    } else if (cmd == 'r' || cmd == 'R') {
      rampTest();
    } else if (cmd == 's' || cmd == 'S') {
      sendThrottlePercent(0);
      Serial.println("Motor parado (0%).");
    } else {
      // tentar interpretar como número de 0..100
      int val = t.toInt();
      if (val < 0) val = 0;
      if (val > 100) val = 100;
      sendThrottlePercent(val);
      Serial.print("Throttle setado para: ");
      Serial.print(val);
      Serial.println("%");
    }
  }
}

// Envia um valor percentual (0..100) convertido para microsegundos
void sendThrottlePercent(int percent) {
  percent = constrain(percent, 0, 100);
  int pulse = map(percent, 0, 100, MIN_PULSE, MAX_PULSE);
  esc.writeMicroseconds(pulse);
}

// Rotina de calibração (procedimento típico):
// - envia MAX por 2s, depois MIN por 2s.
// Nota: muitos ESCs demandam que a bateria seja conectada com o sinal já em MAX para entrar no modo de calibração.
// Se seu ESC exige conectar bateria para calibrar, siga instruções do fabricante. Aqui fazemos algo simples via software.
void calibrateESC() {
  Serial.println("== Calibração ESC ==");
  Serial.println("Sending MAX pulse...");
  esc.writeMicroseconds(MAX_PULSE);
  delay(2000);
  Serial.println("Now sending MIN pulse...");
  esc.writeMicroseconds(MIN_PULSE);
  delay(2000);
  Serial.println("Calibração (sequencia) enviada. Verifique sinais sonoros do ESC.");
  Serial.println("Se seu ESC pede para conectar a bateria durante MAX, desconecte e reconecte bateria conforme instrucoes do ESC.");
  // mantém em MIN
  esc.writeMicroseconds(MIN_PULSE);
}

// Teste de rampa: sobe progressivamente 0% -> 100% -> 0%
void rampTest() {
  Serial.println("== Rampa: 0% -> 100% -> 0% ==");
  // Rampa subida
  int pulse = MIN_PULSE;
  while (pulse < MAX_PULSE) {
    esc.writeMicroseconds(pulse);
    pulse += RAMP_STEP_US;
    delay(RAMP_STEP_MS);
  }
  esc.writeMicroseconds(MAX_PULSE);
  Serial.println("Chegou ao MAX.");
  delay(1500);  // seguro curto: mantém no máximo por 1.5s (sem hélice!)
  // Rampa descida
  pulse = MAX_PULSE;
  while (pulse > MIN_PULSE) {
    esc.writeMicroseconds(pulse);
    pulse -= RAMP_STEP_US;
    delay(RAMP_STEP_MS);
  }
  esc.writeMicroseconds(MIN_PULSE);
  Serial.println("Rampa finalizada. Motor parado (MIN).");
}
