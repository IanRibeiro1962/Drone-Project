// MPU6050.h - Gerenciamento do Sensor MPU6050 (VERSÃO CORRIGIDA)
#ifndef MPU6050_H
#define MPU6050_H

#include "Config.h"

bool setupMPU6050() {
  // Verifica se o MPU6050 está presente
  Wire.beginTransmission(0x68);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("ERRO: MPU6050 não encontrado (código ");
    Serial.print(error);
    Serial.println(")");
    return false;
  }

  // Configuração do sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);  // Sai do sleep mode
  if (Wire.endTransmission() != 0) {
    Serial.println("ERRO: Falha configurando PWR_MGMT_1");
    return false;
  }

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);  // DLPF para ~42Hz (melhor para controle)
  if (Wire.endTransmission() != 0) {
    Serial.println("ERRO: Falha configurando DLPF");
    return false;
  }

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);  // Acelerômetro ±8g
  if (Wire.endTransmission() != 0) {
    Serial.println("ERRO: Falha configurando ACCEL_CONFIG");
    return false;
  }

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);  // Giroscópio ±500°/s
  if (Wire.endTransmission() != 0) {
    Serial.println("ERRO: Falha configurando GYRO_CONFIG");
    return false;
  }

  Serial.println("MPU6050 configurado com sucesso!");
  return true;
}

bool readMPU6050() {
  // Lê acelerômetro
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {  // false mantém conexão ativa
    return false;
  }

  if (Wire.requestFrom(0x68, 6, true) != 6) {  // 6 bytes do acelerômetro
    return false;
  }

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Lê giroscópio
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(0x68, 6, true) != 6) {  // 6 bytes do giroscópio
    return false;
  }

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Conversão para unidades reais
  // Giroscópio: 65.5 LSB/°/s para escala ±500°/s (datasheet)
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Acelerômetro: 4096 LSB/g para escala ±8g (datasheet)
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  // Cálculo dos ângulos a partir do acelerômetro
  // Roll: arco tangente da componente Y pela resultante XZ
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  // Pitch: arco tangente da componente X pela resultante YZ (negativo por convenção)
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);

  return true;
}

void componentsCalibrate() {
  unsigned long calibStart = micros();

  // Zera valores de calibração
  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;

  int successfulReads = 0;

  // Coleta várias amostras para calibração
  for (int i = 0; i < sampleCount; i++) {
    if (readMPU6050()) {
      RateCalibrationRoll += RateRoll;
      RateCalibrationPitch += RatePitch;
      RateCalibrationYaw += RateYaw;
      successfulReads++;
    }
    delay(1);  // Pequena pausa entre leituras
  }

  unsigned long calibEnd = micros();

  if (successfulReads > 0) {
    // Calcula tempo médio de amostragem
    Ts_calib = (calibEnd - calibStart) / (float)(successfulReads * 1000000.0);

    // Calcula médias dos valores de calibração
    RateCalibrationRoll /= successfulReads;
    RateCalibrationPitch /= successfulReads;
    RateCalibrationYaw /= successfulReads;

    Serial.print("Calibração concluída: ");
    Serial.print(successfulReads);
    Serial.println(" leituras válidas");
  } else {
    Serial.println("ERRO: Nenhuma leitura válida durante calibração!");
  }
}

#endif