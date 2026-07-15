/*
  =====================================================================
  LENGAN ROBOT 3 DoF - INVERSE KINEMATICS
  Program Studi Teknik Elektro - Universitas Sanata Dharma
  Disusun Oleh:
    Theodore Galeno Gunadi   - 225114007
    Talita Prima Lestari     - 225114011
    Delpita Laura Br. Galingging - 225114035
    Thomas Dava Mahendra Putra   - 225114038

  Program ini menerima koordinat (x, y, z) end-effector melalui Serial
  Monitor, menghitung sudut tiap sendi (theta1, theta2, theta3) memakai
  Inverse Kinematics, lalu menggerakkan servo Base, Shoulder, dan Elbow
  secara halus (smooth) diikuti aksi end-effector.

  Catatan: Bagian perhitungan calculateIK() melengkapi potongan kode pada
  dokumentasi (Bab 3.4 & 4.2 laporan) mengikuti rumus:
    theta1 = atan2(y, x)
    r      = sqrt(x^2 + y^2)
    a      = sqrt(r^2 + z^2)
    alpha1 = atan2(z, r)
    alpha2 = acos((L1^2 + a^2 - L2^2) / (2*L1*a))
    theta2 = alpha1 + alpha2
    alpha3 = acos((L1^2 + L2^2 - a^2) / (2*L1*L2))
    theta3 = 180 - alpha3
  Hasil ini konsisten dengan Tabel 4.1 dan 4.2 pada laporan.
  =====================================================================
*/

#include <Servo.h>
#include <math.h>

// --- Definisi Objek Servo ---
Servo servoBase;        // Joint 1 (Base) - Pin 4
Servo servoShoulder;    // Joint 2 (Lengan 1) - Pin 5
Servo servoElbow;       // Joint 3 (Lengan 2) - Pin 6
Servo servoEndEffector; // End Effector - Pin 7

// --- Konfigurasi Pin ---
const int PIN_BASE = 4;
const int PIN_SHOULDER = 5;
const int PIN_ELBOW = 6;
const int PIN_ENDEFFECTOR = 7;

// --- Konstanta Robot (panjang lengan, dalam cm) ---
const float L1 = 9.0;   // Panjang link 1 (Shoulder - Elbow)
const float L2 = 8.0;   // Panjang link 2 (Elbow - End Effector)

// Variabel posisi input
float x_in = 0;
float y_in = 0;
float z_in = 0;

// Variabel untuk melacak posisi servo saat ini (agar gerakan smooth)
int currentBasePos = 90;
int currentShoulderPos = 90;
int currentElbowPos = 90;

// Kecepatan Gerakan (semakin besar delay, semakin pelan/halus)
const int SPEED_DELAY = 15;

void setup() {
  Serial.begin(9600);

  // Attach Servo Lengan
  servoBase.attach(PIN_BASE);
  servoShoulder.attach(PIN_SHOULDER);
  servoElbow.attach(PIN_ELBOW);

  // Attach Servo End Effector
  servoEndEffector.attach(PIN_ENDEFFECTOR);

  // Posisi Awal (Home)
  currentBasePos = 90;
  currentShoulderPos = 90;
  currentElbowPos = 90;

  // Tulis posisi awal langsung agar cepat saat boot
  servoBase.write(currentBasePos);
  servoShoulder.write(currentShoulderPos);
  servoElbow.write(currentElbowPos);
  servoEndEffector.write(0); // Posisi awal end effector

  Serial.println("=== Robot Arm Inverse Kinematics (Smooth + End Effector) ===");
  Serial.println("Masukkan koordinat format: x,y,z");
  Serial.println("Contoh: 8,0,9");
  Serial.println("--------------------------------------------");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseInput(input);
  }
}

void parseInput(String data) {
  int comma1 = data.indexOf(',');
  int comma2 = data.indexOf(',', comma1 + 1);

  if (comma1 > 0 && comma2 > 0) {
    x_in = data.substring(0, comma1).toFloat();
    y_in = data.substring(comma1 + 1, comma2).toFloat();
    z_in = data.substring(comma2 + 1).toFloat();

    Serial.print("Input diterima -> X: "); Serial.print(x_in);
    Serial.print(" Y: "); Serial.print(y_in);
    Serial.print(" Z: "); Serial.println(z_in);

    calculateIK(x_in, y_in, z_in);
  } else {
    Serial.println("Format salah! Gunakan: x,y,z");
  }
}

void calculateIK(float x, float y, float z) {
  // --- LOGIKA MATEMATIKA (Inverse Kinematics 3 DoF) ---
  float theta1_rad = atan2(y, x);
  float r = sqrt(x * x + y * y);
  float a = sqrt(r * r + z * z);

  if (a > (L1 + L2)) {
    Serial.println("ERROR: Target di luar jangkauan lengan!");
    return;
  }

  // --- Mencari theta2 (theta2 = alpha1 + alpha2) ---
  float alpha1 = atan2(z, r);
  float cos_alpha2 = (sq(L1) + sq(a) - sq(L2)) / (2 * L1 * a);
  float alpha2 = acos(constrain(cos_alpha2, -1.0, 1.0));
  float theta2_rad = alpha1 + alpha2;

  // --- Mencari theta3 (theta3 = 180 - alpha3) ---
  float cos_alpha3 = (sq(L1) + sq(L2) - sq(a)) / (2 * L1 * L2);
  float alpha3 = acos(constrain(cos_alpha3, -1.0, 1.0));
  float theta3_rad = PI - alpha3;

  // --- Konversi Radian ke Derajat ---
  float theta1_deg = theta1_rad * 180.0 / PI;
  float theta2_deg = theta2_rad * 180.0 / PI;
  float alpha3_deg = theta3_rad * 180.0 / PI; // menyimpan nilai theta3 akhir (derajat)

  // Servo tidak mengenal sudut negatif -> sesuaikan kuadran (contoh: -56,31 + 180 = 123,69)
  float servo1_val = theta1_deg;
  if (servo1_val < 0) {
    servo1_val += 180;
  }

  Serial.print("Target Sudut -> T1: "); Serial.print(servo1_val, 2);
  Serial.print(" | T2: "); Serial.print(theta2_deg, 2);
  Serial.print(" | T3: "); Serial.println(alpha3_deg, 2);

  // --- EKSEKUSI GERAKAN ---
  if (!isnan(servo1_val) && !isnan(theta2_deg) && !isnan(alpha3_deg)) {
    int s1 = constrain((int)round(servo1_val), 0, 180);
    int s2 = constrain((int)round(theta2_deg), 0, 180);
    int s3 = constrain((int)round(alpha3_deg), 0, 180); // Menggunakan alpha3_deg sesuai kode asli

    // 1. Gerakkan Lengan secara Smooth ke posisi target
    moveServosSmooth(s1, s2, s3);

    // 2. Setelah sampai, jalankan aksi End Effector
    runEndEffector();
  }
}

// --- Gerakan Halus (Smooth Movement) ---
void moveServosSmooth(int targetBase, int targetShoulder, int targetElbow) {
  Serial.println("Bergerak halus ke posisi...");

  // Loop sampai semua servo mencapai posisi target
  while (currentBasePos != targetBase || currentShoulderPos != targetShoulder || currentElbowPos != targetElbow) {

    // Update Base
    if (currentBasePos < targetBase) currentBasePos++;
    else if (currentBasePos > targetBase) currentBasePos--;
    servoBase.write(currentBasePos);

    // Update Shoulder
    if (currentShoulderPos < targetShoulder) currentShoulderPos++;
    else if (currentShoulderPos > targetShoulder) currentShoulderPos--;
    servoShoulder.write(currentShoulderPos);

    // Update Elbow
    if (currentElbowPos < targetElbow) currentElbowPos++;
    else if (currentElbowPos > targetElbow) currentElbowPos--;
    servoElbow.write(currentElbowPos);

    // Delay ini menentukan kecepatan (semakin besar semakin lambat/halus)
    delay(SPEED_DELAY);
  }
  Serial.println("Posisi tercapai.");
}

// --- Aksi End Effector (0-180-0) ---
void runEndEffector() {
  Serial.println("Menjalankan End Effector...");

  // Putar dari 0 ke 180
  for (int pos = 0; pos <= 180; pos++) {
    servoEndEffector.write(pos);
    delay(5); // Atur kecepatan putaran
  }

  delay(500); // Tunggu sebentar di posisi 180

  // Putar kembali dari 180 ke 0
  for (int pos = 180; pos >= 0; pos--) {
    servoEndEffector.write(pos);
    delay(5); // Atur kecepatan putaran balik
  }

  Serial.println("End Effector Selesai.");
}
