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

//●❯────────｢ MOTORES ｣────────❮●
const int motorsPins[] = { 18, 19, 26, 25 };  //Pinos do motor
const int numChannels = sizeof(motorsPins) / sizeof(motorsPins[0]);

uint32_t frequencia = 50;  // Frequência de 50 Hz para o ESC
uint8_t resolucao = 16;    // Resolução de 16 bits

uint8_t canalPWM1 = 0;
uint8_t canalPWM2 = 1;

float MotorInput1, MotorInput2;  //Váriavel do PWM onde vai estar o PID
//Usando uma resolução de 16 bits (valores de 0 a 65535),
//1 ms (Vel Minima) Corresponde 3276 e 2 ms a 6553 (Vel Máxima).

//●❯────────｢ GIROSCOPIO ｣────────❮●
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

//●❯────────｢ TEMPO ｣────────❮●
float Ts, lastTime, currentTime;

//●❯────────｢ CONTROLADOR PID ｣────────❮●
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PIDReturn[] = { 0, 0, 0 };

//Desejado
float DesiredAngleRoll, DesiredAnglePitch;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;

//Erro
float ErrorAngleRoll, ErrorAnglePitch;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;

float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;


// Definir ganho feedforward
const float Kff_Roll = 0.5;

// ganho de back-calculation
const float Kb = 0.5;

//Parametros XZ YZ
float PAngleRoll = 5;
float PAnglePitch = PAngleRoll;
float IAngleRoll = 0;
float IAnglePitch = IAngleRoll;
float DAngleRoll = 0;
float DAnglePitch = DAngleRoll;

float PRateRoll = 10;
float PRatePitch = PRateRoll;
float PRateYaw = 2;

float IRateRoll = 0;
float IRatePitch = IRateRoll;
float IRateYaw = 0;

float DRateRoll = 0;
float DRatePitch = DRateRoll;
float DRateYaw = 0;

void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Configura o canal LEDC com a frequência e resolução definidas (motor Esquerda)
  ledcAttachChannel(motorsPins[0], frequencia, resolucao, canalPWM1);
  ledcAttachChannel(motorsPins[1], frequencia, resolucao, canalPWM2);

  // Envia um pulso mínimo para inicializar o ESC (geralmente 1 ms, ou seja, ~3276) (Motor Esquerda)
  ledcWrite(motorsPins[0], 3276);
  ledcWrite(motorsPins[1], 3276);

  //Configurações do Giroscopio
  SetupGir();

  //Calibração dos Componentes
  componentsCalibrate();

  lastTime = micros();
}

void loop() {
  currentTime = micros();
  Ts = (currentTime - lastTime) / 1000000.0;

  //Sinais do Giroscopio
  SinaisGiroscopio();

  //Calbiração do Giroscopio
  RateRoll = -RateCalibrationRoll;
  RatePitch = -RateCalibrationPitch;
  RateYaw = -RateCalibrationYaw;

  DesiredAngleRoll = 0;

  ErrorAngleRoll = DesiredAngleRoll - AngleRoll;

  //CALCULO PID EM RELAÇÃO AO XZ
  //Calculo do controle do Angulo do drone
  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  ErrorRateRoll = DesiredRateRoll - RateRoll;  //Diferença entre o angulo desejado e o valor atual do giroscopio



  float pwmMot1 = map(ErrorRateRoll, -50, 150, 3500, 4000);
  float pwmMot2 = map(ErrorRateRoll, -50, 150, 4000, 3500);


  ledcWrite(motorsPins[0], pwmMot2);
  ledcWrite(motorsPins[1], pwmMot1);

  //Leitura dos Angulso em relação a x e y (Com Filtro de Kalman)

  // Serial.print("Erro1: ");
  // Serial.print(ErrorRateRoll);
  // Serial.print("pwm 1: ");
  // Serial.print(pwmMot1);
  // Serial.print(" Erro 2: ");
  // Serial.print(ErrorRateRoll);
  // Serial.print(" pwm 2: ");
  // Serial.println(pwmMot2);

 Serial.print("Min:-180,Setpoint:"); 
  Serial.print(DesiredAngleRoll);
  Serial.print(",Erro:");
  Serial.print(ErrorRateRoll);
  Serial.print(",Max:180");
  Serial.println();

  lastTime = currentTime;
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

    // Armazena os ângulos para cálculo da variância de Allan
    AccDataXZ[i] = AnglePitch;  // Exemplo: usando AnglePitch
    AccDataYZ[i] = AngleRoll;   // Exemplo: usando AngleRoll

    delay(1);
  }
  unsigned long calibEnd = micros();

  // Calcula o dt médio durante a calibração (em segundos)
  float Ts_calib = (calibEnd - calibStart) / (float)(sampleCount * 1000000.0);

  // Calcula a variância de Allan para cada eixo
  allanVarXZ = computeAllanVariance(AccDataXZ, sampleCount, Ts_calib, 1);
  allanVarYZ = computeAllanVariance(AccDataYZ, sampleCount, Ts_calib, 1);

  // Calcula as médias para a calibração dos giroscópios
  RateCalibrationRoll /= sampleCount;
  RateCalibrationPitch /= sampleCount;
  RateCalibrationYaw /= sampleCount;
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

  //--------------------------------------------------------------------------------
  // NOTA:
  // - Não é necessário configurar os ponteiros de leitura (ex.: registradores 0x3B e 0x43)
  //   aqui, pois isso é feito na função de leitura (SinaisGiroscopio) para definir o endereço
  //   inicial dos dados a serem lidos.
  //--------------------------------------------------------------------------------
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
  // Limites para o sinal do integrador
  const float U_MAX = 1638.5;   // Limite superior
  const float U_MIN = -1638.5;  // Limite inferior

  // Cálculo dos termos proporcional e derivativo
  float Pterm = P * Error;
  float Dterm = D * (Error - PrevError) / Ts;

  // Calcula o termo integral "cru" usando integração trapezoidal
  float Iterm_raw = PrevIterm + I * (Error + PrevError) * Ts / 2;

  // Calcula o sinal de controle bruto (antes da saturação)
  float u_calc = Pterm + Iterm_raw + Dterm;

  // Aplica a saturação para obter o sinal efetivamente aplicado
  float u_sat = u_calc;
  if (u_sat > U_MAX)
    u_sat = U_MAX;
  else if (u_sat < U_MIN)
    u_sat = U_MIN;

  // Implementação do Anti-Windup com integração condicional
  // Se o sinal bruto está fora dos limites e o erro está na direção que aumentaria a saturação,
  // o termo integral não é atualizado para evitar o acúmulo (windup)
  float Iterm;
  if ((u_calc > U_MAX && Error > 0) || (u_calc < U_MIN && Error < 0)) {
    // Não integra se já estiver saturado na direção do erro
    Iterm = PrevIterm;
  } else {
    Iterm = Iterm_raw;
  }

  // Recalcula a saída PID com o Iterm ajustado
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > U_MAX)
    PIDOutput = U_MAX;
  else if (PIDOutput < U_MIN)
    PIDOutput = U_MIN;

  // Atualiza o array de retorno do PID
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                          FILTRO DE KALMAN
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
void FiltroKalman(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, float allanVar) {
  KalmanState = KalmanState + Ts * KalmanInput;                                   //Predição do Estado (Eq 1)
  KalmanUncertainty = KalmanUncertainty + Ts * Ts * 4 * 4;                        //Predição da Incerteza (Eq 2)
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + allanVar);  //Calculo do Ganho de Kalman (Eq 3)
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);     //Atualização da Medição (Eq 4)
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;                       //Atualização da Incerteza (Eq 5)

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
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
