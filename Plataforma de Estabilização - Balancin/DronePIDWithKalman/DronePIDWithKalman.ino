// Inclusão das Bibliotecas
#include <Wire.h>

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                              VARIAVEIS
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚

//●❯────────｢ VARIÂNCIA DE ALLAN ｣────────❮●
const int sampleCount = 2000;  // Número de amostras para cálculo da variância de Allan
float AccDataXZ[sampleCount];  // Armazena os valores medidos do giroscópio
float AccDataYZ[sampleCount];  // Armazena os valores medidos do giroscópio
float allanVarXZ, allanVarYZ;

//●❯────────｢ PARAMETROS DO FILTRO DE KALMAN ｣────────❮●
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };

//●❯────────｢ ANALISADOR PARA TU ZIGGLER NICHOLS ｣────────❮●
typedef struct {
  // Configuração
  float threshold;   // Limiar de oscilação
  uint8_t minPeaks;  // Mínimo de picos para cálculo

  // Estado interno
  float lastValue;
  unsigned long lastPeakTime;
  uint16_t peakCount;
  float periodSum;

  // Resultados
  float Ku;
  float Pu;
} KuPuAnalyzer;

// Declarações antecipadas (prototypes)
void initAnalyzer(KuPuAnalyzer* analyzer, float threshold, uint8_t minPeaks);
void updateAnalysis(KuPuAnalyzer* analyzer, float currentValue, float currentGain);

KuPuAnalyzer rateAnalysis, angleAnalysis;


//●❯────────｢ MOTORES ｣────────❮●
const int motorsPins[] = { 18, 19 };  //Pinos do motor
const int numChannels = sizeof(motorsPins) / sizeof(motorsPins[0]);

uint32_t frequencia = 50;  // Frequência de 50 Hz para o ESC
uint8_t resolucao = 16;    // Resolução de 16 bits

uint8_t canalPWM1 = 0;
uint8_t canalPWM2 = 1;

float MotorInput1, MotorInput2;  //Váriavel do PWM onde vai estar o PID
//Usando uma resolução de 16 bits (valores de 0 a 65535),
//1 ms (Vel Minima) Corresponde 3276 e 2 ms a 6553 (Vel Máxima).


//●❯────────｢ ESCs ｣────────❮●
const uint16_t ESC_PULSE_MIN = 3276;     // 1 ms em 16 bits (65535/20)
const uint16_t ESC_PULSE_MAX = 4000;     // 2 ms em 16 bits
const uint16_t CALIB_DELAY_HIGH = 3000;  // ms para manter o pulso máximo
const uint16_t CALIB_DELAY_LOW = 3000;   // ms para manter o pulso mínimo


//●❯────────｢ GIROSCOPIO ｣────────❮●
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

//●❯────────｢ TEMPO ｣────────❮●
float Ts, lastTime, currentTime, Ts_calib;
double sumTs = 0;
const float CONTROL_TS = 0.01f; // 10ms = 100Hz (PERÍODO FIXO)
unsigned long lastControlTime = 0;

//●❯────────｢ CONTROLADOR PID ｣────────❮●
float InputRoll, InputThrottle;
float PIDReturn[] = { 0, 0, 0 };

//Desejado
float DesiredAngleRoll;
float DesiredRateRoll;

//Erro
float ErrorAngleRoll;
float ErrorRateRoll, ErrorRatePitch;

float PrevErrorAngleRoll;
float PrevItermAngleRoll;

float PrevErrorRateRoll;
float PrevItermRateRoll;

//●❯────────｢ LIMITES ENTRE LAÇOS ｣────────❮●
const float MAX_ANGLE_OUTPUT = 100.0f;   // °/s máximo para setpoint do rate
const float MAX_RATE_OUTPUT = 400.0f;    // Limite para saída do rate loop

//Parametros XZ YZ
float PuAngle = 0;
float KuAngle = 1;

float PAngleRoll = KuAngle;
float IAngleRoll = 0;
float DAngleRoll = 0;


//Parametros Para o amortecedora
float PuRate = 1;
float KuRate = 1;

float PRateRoll = 0.6 * KuRate;
float IRateRoll = (1.2 * KuRate) / PuRate;
float DRateRoll = 0.075 * KuRate * PuRate;

void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Configura o canal LEDC com a frequência e resolução definidas (motor Esquerda)
  ledcAttachChannel(motorsPins[0], frequencia, resolucao, canalPWM1);
  ledcAttachChannel(motorsPins[1], frequencia, resolucao, canalPWM2);

  //Calibração dos ESCs
  calibrateESCs();

  // Envia um pulso mínimo para inicializar o ESC (geralmente 1 ms, ou seja, ~3276) (Motor Esquerda)
  ledcWrite(motorsPins[0], 3276);
  ledcWrite(motorsPins[1], 3276);

  //Configurações do Giroscopio
  SetupGir();

  //Calibração dos Componentes
  lastTime = micros();
  componentsCalibrate();

  // COLETA DADOS PARA ALLAN VARIANCE (CORREÇÃO 2)
  collectAllanData();
  allanVarXZ = computeAllanVariance(AccDataXZ, sampleCount, Ts_calib, 50);

  // Inicializa Kalman com ângulo atual (CORREÇÃO 1)
  KalmanAngleRoll = AngleRoll;
  KalmanAnglePitch = AnglePitch;

  // Inicializa analisadores
  initAnalyzer(&rateAnalysis, 5.0, 5);   // Threshold 2.0, 5 picos
  initAnalyzer(&angleAnalysis, 1.0, 5);  // Threshold 2.0, 5 picos

  lastControlTime = micros();
}

void loop() {
  currentTime = micros();
  
  // CORREÇÃO 3: PERÍODO DE AMOSTRAGEM FIXO
  if (currentTime - lastControlTime >= CONTROL_TS * 1000000) {
    Ts = CONTROL_TS; // Ts fixo!

    // Atualiza análises
    updateAnalysis(&rateAnalysis, RateRoll, PRateRoll);
    updateAnalysis(&angleAnalysis, AngleRoll, PAngleRoll);

    //Sinais do Giroscopio
    SinaisGiroscopio();

    //Calbiração do Giroscopio
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    // CORREÇÃO 1: FILTRO KALMAN CORRIGIDO (sem parâmetros)
    FiltroKalman();

    DesiredAngleRoll = 0;
    ErrorAngleRoll = KalmanAngleRoll - DesiredAngleRoll;

    //CALCULO PID EM RELAÇÃO AO XZ
    //Calculo do controle do Angulo do drone por meio do Acelerometro (Laço Externo)
    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
    
    // CORREÇÃO 4: LIMITE ENTRE LAÇOS
    DesiredRateRoll = constrain(PIDReturn[0], -MAX_ANGLE_OUTPUT, MAX_ANGLE_OUTPUT);
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    ErrorRateRoll = DesiredRateRoll - RateRoll;  //Diferença entre o angulo desejado e o valor atual do giroscopio

    //Calculo da correção do angulo por meio do giroscopio (Laço Interno)
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    
    // CORREÇÃO 4: LIMITE ENTRE LAÇOS  
    InputRoll = constrain(PIDReturn[0], -MAX_RATE_OUTPUT, MAX_RATE_OUTPUT);
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    InputThrottle = 3600;

    MotorInput1 = (InputThrottle + InputRoll);
    MotorInput2 = (InputThrottle - InputRoll);

    MotorInput1 = constrain(MotorInput1, ESC_PULSE_MIN, ESC_PULSE_MAX);
    MotorInput2 = constrain(MotorInput2, ESC_PULSE_MIN, ESC_PULSE_MAX);

    ledcWrite(motorsPins[0], MotorInput1);
    ledcWrite(motorsPins[1], MotorInput2);

    // DEBUG PARA ACHAR O PERIODO DE OSCILAÇÕES
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {  // Atualiza a cada 500ms
      Serial.println("=== Parâmetros Críticos ===");

      //Analise do Angulo
      if (angleAnalysis.Pu > 0) {
        Serial.print("Angle Loop: Ku=");
        Serial.print(angleAnalysis.Ku);
        Serial.print(" Pu=");
        Serial.println(angleAnalysis.Pu, 6);
      }

      lastPrint = millis();
    }

    lastControlTime = currentTime;
  }
  
  lastTime = currentTime;
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                    ESTRUTURA PARA MEDIÇÃO KU/PU
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚

//●❯────────｢ INICIALIZADOR ｣────────❮●
void initAnalyzer(KuPuAnalyzer* analyzer, float threshold, uint8_t minPeaks) {
  analyzer->threshold = threshold;
  analyzer->minPeaks = minPeaks;
  analyzer->lastValue = 0;
  analyzer->lastPeakTime = 0;
  analyzer->peakCount = 0;
  analyzer->periodSum = 0;
  analyzer->Ku = 0;
  analyzer->Pu = 0;
}

//●❯────────｢ FUNÇÃO DE ANÁLISE ｣────────❮●
void updateAnalysis(KuPuAnalyzer* analyzer, float currentValue, float currentGain) {
  // Detecção de borda de subida
  if ((currentValue > analyzer->threshold) && (analyzer->lastValue <= analyzer->threshold)) {
    unsigned long now = micros();

    if (analyzer->lastPeakTime != 0) {  // Ignora primeiro pico
      float period = (now - analyzer->lastPeakTime) / 1000000.0f;
      analyzer->periodSum += period;
      analyzer->peakCount++;

      if (analyzer->peakCount >= analyzer->minPeaks) {
        analyzer->Pu = analyzer->periodSum / (analyzer->peakCount - 1);
        analyzer->Ku = currentGain;  // Captura o ganho crítico
        analyzer->periodSum = 0;
        analyzer->peakCount = 0;
      }
    }
    analyzer->lastPeakTime = now;
  }
  analyzer->lastValue = currentValue;
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                   CONFIGURAÇÕES DOS ESCS
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
void calibrateESCs() {
  Serial.println("=== Iniciando calibração dos ESCs ===");

  // 1) envia pulso máximo para todos os ESCs
  Serial.println("-> Pulso MÁXIMO (2 ms)");
  for (int i = 0; i < numChannels; i++) {
    ledcWrite(motorsPins[i], ESC_PULSE_MAX);
  }
  delay(CALIB_DELAY_HIGH);

  // 2) envia pulso mínimo para todos os ESCs
  Serial.println("-> Pulso MÍNIMO (1 ms)");
  for (int i = 0; i < numChannels; i++) {
    ledcWrite(motorsPins[i], ESC_PULSE_MIN);
  }
  delay(CALIB_DELAY_LOW);

  Serial.println("=== Calibração concluída! ===");
  delay(500);
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                     CALIBRAÇÃO DOS COMPONENTES
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
void componentsCalibrate() {
  unsigned long calibStart = micros();

  // Inicializa as somas para calibração
  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;

  // Coleta as amostras
  for (int i = 0; i < sampleCount; i++) {
    SinaisGiroscopio();  // Função que faz a leitura dos sensores e atualiza RateRoll, RatePitch, RateYaw e os ângulos

    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;

    delay(1);
  }
  unsigned long calibEnd = micros();

  // Calcula o dt médio durante a calibração (em segundos)
  Ts_calib = (calibEnd - calibStart) / (float)(sampleCount * 1000000.0);

  // Calcula as médias para a calibração dos giroscópios
  RateCalibrationRoll /= sampleCount;
  RateCalibrationPitch /= sampleCount;
  RateCalibrationYaw /= sampleCount;
}

//●❯────────｢ COLETA DADOS ALLAN (CORREÇÃO 2) ｣────────❮●
void collectAllanData() {
  Serial.println("Coletando dados para Allan Variance...");
  for (int i = 0; i < sampleCount; i++) {
    SinaisGiroscopio();
    AccDataXZ[i] = RateRoll;  // Coleta dados reais do giroscópio
    AccDataYZ[i] = RatePitch;
    delay(1); // Manter mesma taxa de amostragem da calibração
  }
  Serial.println("Coleta de dados Allan concluída!");
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                      CONFIGURAÇÃO DO GIROSCOPIO
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
void SetupGir() {
  //--------------------------------------------------------------------------------
  // 1. Despertar o sensor e configurar a fonte de clock
  // PWR_MGMT_1 (registrador 0x6B):
  // - Escrevendo 0x00, o sensor é despertado do modo sleep
  //   e o oscilador interno é selecionado como fonte de clock.
  //--------------------------------------------------------------------------------
  Wire.beginTransmission(0x68);  // Inicia transmissão com o MPU6050 (endereço I2C 0x68)
  Wire.write(0x6B);              // Seleciona o registrador PWR_MGMT_1
  Wire.write(0x00);              // Desativa o modo sleep e configura o clock
  Wire.endTransmission();

  //--------------------------------------------------------------------------------
  // 2. Configurar o Filtro Digital Passa-Baixa (DLPF)
  // CONFIG (registrador 0x1A):
  // - O valor escrito define a frequência de corte do filtro.
  //   Por exemplo, escrever 0x05 pode configurar uma frequência de corte em torno de 10 Hz
  //   para os giroscópios (verifique a tabela do datasheet para confirmar o valor exato).
  //--------------------------------------------------------------------------------
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // Seleciona o registrador CONFIG (DLPF)
  Wire.write(0x05);  // Define o valor do DLPF (ajuste conforme necessário)
  Wire.endTransmission();

  //--------------------------------------------------------------------------------
  // 3. Configurar o acelerômetro
  // ACCEL_CONFIG (registrador 0x1C):
  // - O valor escrito aqui define a faixa de medição do acelerômetro.
  //   No exemplo, 0x10 normalmente configura a faixa para ±8g.
  //--------------------------------------------------------------------------------
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);  // Seleciona o registrador ACCEL_CONFIG
  Wire.write(0x10);  // Configura a faixa do acelerômetro para ±8g (verifique no datasheet)
  Wire.endTransmission();

  //--------------------------------------------------------------------------------
  // 4. Configurar o giroscópio
  // GYRO_CONFIG (registrador 0x1B):
  // - Este registrador define a faixa de medição do giroscópio.
  //   No exemplo, 0x08 configura a faixa para ±500°/s.
  //--------------------------------------------------------------------------------
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  // Seleciona o registrador GYRO_CONFIG
  Wire.write(0x08);  // Configura a faixa do giroscópio para ±500°/s (0x08 = 00001000 em binário)
  Wire.endTransmission();
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                      LEITURA DO GIROSCOPIO
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
void SinaisGiroscopio(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);  // false para manter a conexão e enviar um repeated start

  Wire.requestFrom(0x68, 6, true);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 6, true);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                          EQUAÇÃO DO PID
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  // Cálculo dos termos proporcional e derivativo
  float Pterm = P * Error;
  float Dterm = D * (Error - PrevError) / Ts;

  float Iterm = PrevIterm + I * (Error + PrevError) * Ts / 2;
  Iterm = constrain(Iterm, -1638.5, 1638.5);

  // saída PID saturada:
  float PIDOutput = Pterm + Iterm + Dterm;

  // Atualiza o array de retorno do PID
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                          FILTRO DE KALMAN (CORREÇÃO 1)
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
void FiltroKalman() {
  // PREDIÇÃO
  KalmanAngleRoll = KalmanAngleRoll + Ts * RateRoll;
  KalmanUncertaintyAngleRoll = KalmanUncertaintyAngleRoll + Ts * Ts * allanVarXZ;
  
  // ATUALIZAÇÃO
  float KalmanGain = KalmanUncertaintyAngleRoll / (KalmanUncertaintyAngleRoll + allanVarXZ);
  KalmanAngleRoll = KalmanAngleRoll + KalmanGain * (AngleRoll - KalmanAngleRoll);
  KalmanUncertaintyAngleRoll = (1 - KalmanGain) * KalmanUncertaintyAngleRoll;

  Kalman1DOutput[0] = KalmanAngleRoll;
  Kalman1DOutput[1] = KalmanUncertaintyAngleRoll;
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                          VARIÂNCIA DE ALLAN
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
float computeAllanVariance(float* data, int n, float Ts, int clusterSize) {
  int M = n / clusterSize;
  if (M < 2) return 0;

  float sum = 0;
  for (int i = 0; i < M - 1; i++) {
    float avg1 = 0, avg2 = 0;
    for (int j = 0; j < clusterSize; j++) {
      avg1 += data[i * clusterSize + j];
      avg2 += data[(i + 1) * clusterSize + j];
    }
    avg1 /= clusterSize;
    avg2 /= clusterSize;
    sum += (avg2 - avg1) * (avg2 - avg1);
  }

  float tau = clusterSize * Ts;  // Tempo de integração
  return (sum / (2 * (M - 1))) / (tau * tau);
}