#include <Wire.h>


//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                              VARIAVEIS
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚

//Variância de Allan
const int sampleCount = 2000;  // Número de amostras para cálculo da variância de Allan
float AccDataXZ[sampleCount];  // Armazena os valores medidos do giroscópio
float AccDataYZ[sampleCount];  // Armazena os valores medidos do giroscópio

float allanVarXZ, allanVarYZ;

//Váriaveis
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };

float Ts, lastTime, currentTime;


void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  SetupGir();//Configurações do Giroscopio
  delay(250);

  // Realiza a calibração e calcula Ts médio durante a calibração
  float Ts_calib = componentsCalibrate();

  // Se desejar, plote a curva de Allan para visualizar os dados
  plotAllanCurve(Ts_calib);

  Serial.print("Allan XZ [°] ");
  Serial.print(allanVarXZ);
  Serial.print(" Allan YZ [°] ");
  Serial.println(allanVarYZ);

  lastTime = micros();
}


void loop() {
  currentTime = micros();
  Ts = (currentTime - lastTime) / 1000000.0;

  gyro_signals();

  //Leitura com filtro de Kalman
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  //Aplicação do Filtro de Kalman no eixo XZ
  FiltroKalman(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll, allanVarXZ);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  //Aplicação do Filtro de Kalman no eixo YZ
  FiltroKalman(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch, allanVarYZ);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];


  //Leitura dos Angulso em relação a x e y (Com Filtro de Kalman)

  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(KalmanAnglePitch);


  lastTime = currentTime;
}


//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                     CALIBRAÇÃO DOS COMPONENTES
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
float componentsCalibrate() {
  unsigned long calibStart = micros();

  // Zera os acumuladores de calibração
  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;

  // Coleta sampleCount amostras
  for (int i = 0; i < sampleCount; i++) {
    gyro_signals();  // Atualiza RateRoll, RatePitch, RateYaw, AngleRoll e AnglePitch

    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;

    // Armazena os ângulos para cálculo da variância de Allan
    AccDataXZ[i] = AnglePitch;  // Exemplo: usando AnglePitch para eixo XZ
    AccDataYZ[i] = AngleRoll;   // Exemplo: usando AngleRoll para eixo YZ

    delay(1);
  }
  unsigned long calibEnd = micros();

  // Calcula o dt médio em segundos durante a calibração
  float Ts_calib = (calibEnd - calibStart) / (sampleCount * 1000000.0);

  // Calcula as variâncias de Allan
  allanVarXZ = AllanVariance(AccDataXZ, sampleCount, Ts_calib, 1);
  allanVarYZ = AllanVariance(AccDataYZ, sampleCount, Ts_calib, 1);

  // Calcula a média dos giroscópios para compensação do drift
  RateCalibrationRoll /= sampleCount;
  RateCalibrationPitch /= sampleCount;
  RateCalibrationYaw /= sampleCount;

  // Imprime os valores de Allan para verificação
  Serial.print("Allan XZ [°]: ");
  Serial.println(allanVarXZ, 6);
  Serial.print("Allan YZ [°]: ");
  Serial.println(allanVarYZ, 6);

  return Ts_calib;
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//                  PLOTAR GRÁFICO DA VARIANCIA DE ALLAN
//┖•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┚
void plotAllanCurve(float Ts_calib) {
  Serial.println("tau,allanVarXZ");  // Cabeçalho para facilitar a importação dos dados
  // Varra clusterSize de 1 até um valor desejado (por exemplo, 50)
  for (int clusterSize = 1; clusterSize <= 50; clusterSize++) {
    float tau = clusterSize * Ts_calib;
    float allanVar = AllanVariance(AccDataXZ, sampleCount, Ts_calib, clusterSize);
    //Leitura dos valores da Variancia de Allan
    /*
    Serial.print(tau, 6);
    Serial.print(",");
    Serial.println(allanVar, 6);
        */
    delay(100);  // Pequeno delay para separar as linhas

  }
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
  Wire.beginTransmission(0x68);     // Inicia transmissão com o MPU6050 (endereço I2C 0x68)
  Wire.write(0x6B);                 // Seleciona o registrador PWR_MGMT_1
  Wire.write(0x00);                 // Desativa o modo sleep e configura o clock
  Wire.endTransmission();

  //--------------------------------------------------------------------------------
  // 2. Configurar o Filtro Digital Passa-Baixa (DLPF)
  // CONFIG (registrador 0x1A):
  // - O valor escrito define a frequência de corte do filtro.
  //   Por exemplo, escrever 0x05 pode configurar uma frequência de corte em torno de 10 Hz
  //   para os giroscópios (verifique a tabela do datasheet para confirmar o valor exato).
  //--------------------------------------------------------------------------------
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);                 // Seleciona o registrador CONFIG (DLPF)
  Wire.write(0x05);                 // Define o valor do DLPF (ajuste conforme necessário)
  Wire.endTransmission();

  //--------------------------------------------------------------------------------
  // 3. Configurar o acelerômetro
  // ACCEL_CONFIG (registrador 0x1C):
  // - O valor escrito aqui define a faixa de medição do acelerômetro.
  //   No exemplo, 0x10 normalmente configura a faixa para ±8g.
  //--------------------------------------------------------------------------------
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                 // Seleciona o registrador ACCEL_CONFIG
  Wire.write(0x10);                 // Configura a faixa do acelerômetro para ±8g (verifique no datasheet)
  Wire.endTransmission();

  //--------------------------------------------------------------------------------
  // 4. Configurar o giroscópio
  // GYRO_CONFIG (registrador 0x1B):
  // - Este registrador define a faixa de medição do giroscópio.
  //   No exemplo, 0x08 configura a faixa para ±500°/s.
  //--------------------------------------------------------------------------------
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                 // Seleciona o registrador GYRO_CONFIG
  Wire.write(0x08);                 // Configura a faixa do giroscópio para ±500°/s (0x08 = 00001000 em binário)
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
void gyro_signals(void) {

  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
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



  //Leitura com os eixos (Normal)
  /*
    Serial.print("Acceleration X [g]= ");
    Serial.print(AccX);
    Serial.print(" Acceleration Y [g]= ");
    Serial.print(AccY);
    Serial.print(" Acceleration Z [g]= ");
    Serial.print(AccZ);
    Serial.println(" ");
    delay(50);
  */

  //Leitura dos angulos em relação a x e y (Acelerometro)
  /*
  Serial.print("Angle XZ [°]= ");
  Serial.print(AngleRollAce);
  Serial.print(" Angle YZ [°]= ");
  Serial.println(AnglePitchAce);
  */
}

//┎•━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━ • ━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━━┅━┅━•┒
//               FILTRO DE KALMAN COM VARIÂNCIA DE ALLAN
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
float AllanVariance(float *data, int n, float Ts, int clusterSize) {
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