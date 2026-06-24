/* * Revisi Final Self Balancing Robot
 * Integrasi MPU6050 & L298N dengan PID Control
 * Fitur Tambahan: Pencari Tu (Ultimate Period) untuk Tuning Ziegler-Nichols 2
 * Revisi: Penambahan Output untuk Serial Plotter
 */

#include <math.h>
#include <I2Cdev.h>
#include "Wire.h"
#include <MPU6050.h>
#define RESTRICT_PITCH

// --- Kalman filter variables ---
MPU6050 accelgyro;
float Qacc, Qgyro, R;
float Xkacc, Xkpacc, Xkgyro, Pkp00, Pkp01, dXgyro;
float Pkp[2][2], KG[2], Wk;
unsigned long dtact;
double dtlast, dt;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t accX, accY, accZ;
double gyroY, gyroYlast, gyroYangle;
double kalmanY, PVpitch, pitch;
float New_PVpitch;

// --- Driver motor pins (WIRING AKTUAL TERBARU) ---
const int enA = 10;  // PWM Motor A
const int in1 = 9;
const int in2 = 8;
const int enB = 5;   // PWM Motor B
const int in3 = 7;
const int in4 = 6;

// --- PID Variables ---
float Kp, Ki, Kd;
float Sv, Pid;
float et, et_1, eint, eint_1, edif, eint_update;
int Mv;

// --- Variabel untuk Deteksi Puncak (Ziegler-Nichols 2) ---
float last_PVpitch = 0;
unsigned long lastPeakTime = 0;
unsigned long Tu = 0; 
bool isRising = false;

// --- Variabel untuk Interval Tampilan Plotter ---
unsigned long lastPrintTime = 0;
const int printInterval = 50; // Update grafik setiap 50 milidetik

// Fungsi Set Angle Kalman
void setAngleY(float pitch) {
  Xkacc = pitch;
}
  
float kalman_gyro() {
  return dXgyro;
}

// Fungsi Hitung Kalman Filter
float kalman_calculation(float Xtacc, float Xtgyro, float dt) {
  dXgyro = Xtgyro - Xkgyro;
  Xkacc = Xkacc + (dt * dXgyro) + Wk;
  Pkp[0][0] = Pkp[0][0] + (dt * (dt * Pkp[1][1] - Pkp[0][1] - Pkp[1][0] + Qacc));
  Pkp[0][1] = Pkp[0][1] - (dt * Pkp[1][1]);
  Pkp[1][0] = Pkp[1][0] - (dt * Pkp[1][1]);
  Pkp[1][1] = Pkp[1][1] + (dt * Qgyro);
  KG[0] = Pkp[0][0] / (Pkp[0][0] + R);
  KG[1] = Pkp[1][0] / (Pkp[0][0] + R);
  Xkacc = Xkacc + (KG[0] * (Xtacc - Xkacc));
  Xkgyro = Xkgyro + (KG[1] * (Xtacc - Xkacc));
  Pkp00 = Pkp[0][0];
  Pkp01 = Pkp[0][1];
  Pkp[0][0] = Pkp[0][0] - (KG[0] * Pkp00);
  Pkp[0][1] = Pkp[0][1] - (KG[0] * Pkp01);
  Pkp[1][0] = Pkp[1][0] - (KG[1] * Pkp00);
  Pkp[1][1] = Pkp[1][1] - (KG[1] * Pkp01);
  return Xkacc;
}

// Fungsi Kendali Motor H-Bridge
void Motor_Control(int speedMv) {
  // Matikan motor jika nilai PID 0
  if (speedMv == 0) {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    return;
  }

  // Batasi maksimal PWM ke 255
  int pwm = abs(speedMv);
  if (pwm > 255) pwm = 255;

  if (speedMv > 0) {
    // Gerak Maju
    analogWrite(enA, pwm);
    analogWrite(enB, pwm);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } 
  else if (speedMv < 0) {
    // Gerak Mundur
    analogWrite(enA, pwm);
    analogWrite(enB, pwm);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
}

void setup() {
  // Setup Kalman Filter Init
  Qacc = 0.002;
  Qgyro = 0.004;
  R = 0.4;
  
  Wire.begin();
  Serial.begin(9600);
  
  accelgyro.initialize();
  
  // Setup Nilai Konstanta PID AWAL UNTUK MENCARI Ku
  // Untuk metode ZN2, set Ki dan Kd ke 0.
  // Naikkan Kp pelan-pelan sampai robot berosilasi (bergetar maju-mundur) secara stabil
  Kp = 30; // Ubah nilai ini saat mencari Ku
  Ki = 0.0007;  
  Kd = 0.5;

  et_1 = 0;
  eint_1 = 0;

  // Setup Pin Motor
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  
  // Memastikan motor mati saat awal menyala
  Motor_Control(0); 
}

void loop() {
  // Setpoint (Titik Seimbang), idealnya 0 derajat
  Sv = 0; 
  
  // Hitung dt (Delta Time) secara real-time untuk akurasi integral/derivatif
  dtlast = dtact;
  dtact = millis();
  dt = (dtact - dtlast) / 1000.0; // Ubah ke detik

  // Mencegah error pembagian dengan nol saat looping pertama
  if (dt <= 0) dt = 0.01; 

  // Pembacaan Sensor MPU6050
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accX = ax; 
  accZ = az;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  gyroY = gy / 131.0;
  
  // Eksekusi Kalman Filter
  if ((pitch < -90 && kalmanY > 90) || (pitch > 90 && kalmanY < -90)) {
    setAngleY(pitch);
    kalmanY = pitch;
    gyroYangle = pitch;
  } else {
    kalmanY = kalman_calculation(pitch, gyroY, dt);
  }
    
  gyroYangle = gyroYangle + (kalman_gyro() * dt);

  if (gyroYangle < -180 || gyroYangle > 180) {
    gyroYangle = kalmanY;
  }
  
  New_PVpitch = kalmanY;

  // FAILSAFE: Jika robot jatuh melebihi sudut 40 derajat, matikan motor
  if (New_PVpitch > 40 || New_PVpitch < -40) {
    Pid = 0;
    et_1 = 0;   // Reset error history
    eint_1 = 0; // Reset windup
  } 
  else {
    // --- PERHITUNGAN PID ---
    
    // 1. Proportional (Error)
    et = Sv - New_PVpitch;
    
    // 2. Integral
    eint_update = ((et + et_1) * dt) / 2.0;
    eint = eint_1 + eint_update;
    
    // 3. Derivative
    edif = (et - et_1) / dt;
    
    // Total PID
    Pid = (Kp * et) + (Ki * eint) + (Kd * edif);

    // Update error history untuk perhitungan loop selanjutnya
    et_1 = et;
    eint_1 = eint;
  }

  // Kirim nilai PID ke Motor
  Motor_Control(Pid);

  // --- ALGORITMA PENCARI Tu (ZIEGLER-NICHOLS 2) ---
  
  // Deteksi jika sudut sedang bergerak naik
  if (New_PVpitch > last_PVpitch) {
    isRising = true; 
  } 
  // Jika sebelumnya naik, tapi sekarang turun, berarti ini adalah Puncak (Peak)
  else if (New_PVpitch < last_PVpitch && isRising) {
    unsigned long currentPeakTime = millis();
    
    // Filter noise: Abaikan getaran sangat kecil/cepat (misal < 100ms)
    if ((currentPeakTime - lastPeakTime) > 100) { 
      Tu = currentPeakTime - lastPeakTime; // Jarak antar puncak (Ini adalah Tu dalam milidetik)
      lastPeakTime = currentPeakTime;
      
      // TEKS DIMATIKAN SEMENTARA AGAR SERIAL PLOTTER BISA MEMBACA GRAFIK
      // Serial.print("WaktuPuncak(ms): ");
      // Serial.print(currentPeakTime);
      // Serial.print(" \t Tu(ms): ");
      // Serial.println(Tu);
    }
    isRising = false; // Reset status untuk mencari puncak berikutnya
  }
  
  // Simpan nilai sudut saat ini untuk perbandingan di loop berikutnya
  last_PVpitch = New_PVpitch;

  // --- SERIAL PLOTTER OUTPUT ---
  // Mengirim data murni angka setiap interval waktu tertentu
  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();
    
    // Format: Setpoint(0), SudutAktual, OutputPID
    Serial.print(Sv);           // Garis 1: Target seimbang (0 derajat)
    Serial.print(",");
    Serial.print(New_PVpitch);  // Garis 2: Sudut aktual kemiringan robot
    Serial.print(",");
    Serial.println(Pid);        // Garis 3: Nilai PWM yang dihasilkan PID
    
    // TIPS: Jika garis PID membuat skala grafik sudut jadi tidak terlihat (mendatar),
    // berikan "//" di depan Serial.print(",") dan Serial.println(Pid);
  }
}