# Lengan Robot 3 DoF Inverse Kinematics

Repository ini berisi laporan, kode program, dan dokumentasi pengujian untuk proyek
**Lengan Robot Industri 3 DoF (Degree of Freedom)** berbasis Arduino Uno R3.
Robot digerakkan menggunakan pendekatan *Inverse Kinematics*: pengguna memasukkan
koordinat target end-effector `(x, y, z)`, lalu sistem menghitung sudut tiap sendi
(`θ1`, `θ2`, `θ3`) dan menggerakkan motor servo secara halus (*smooth movement*).

## Tim Penyusun
Program Studi Teknik Elektro — Fakultas Sains dan Teknologi — Universitas Sanata Dharma, Yogyakarta (2025)

| Nama | NIM |
|---|---|
| Theodore Galeno Gunadi | 225114007 |
| Talita Prima Lestari | 225114011 |
| Delpita Laura Br. Galingging | 225114035 |
| Thomas Dava Mahendra Putra | 225114038 |

Dosen Pengampu: Regina Chelinia Erianda Putri, M.T.

## Struktur Repository
```
robot-lengan-3dof/
├── README.md
├── LICENSE
├── src/
│   └── robot_arm_3dof.ino     # Program utama Arduino (Inverse Kinematics + kontrol servo)
├── docs/
│   └── Laporan_Lengan_Robot_3DoF.pdf   # Laporan lengkap (teori, rancangan, analisis error)
└── images/
    ├── uji_koordinat_1_x8_y0_z9.jpeg
    ├── uji_koordinat_2_x6_y2_z12.jpeg
    └── uji_koordinat_3_x2_y3_z5.jpeg
```

## Tujuan
1. Menerapkan konsep *inverse kinematic* untuk mengatur posisi end-effector.
2. Merancang hardware dan software untuk menggerakkan lengan robot.
3. Menguji performa sistem melalui implementasi dan analisis pergerakan.

## Alat dan Bahan
- Akrilik (base)
- Stik es krim (link lengan)
- Lem
- Busur (untuk kalibrasi/pengukuran sudut)
- Kabel jumper
- Arduino Uno R3
- Arduino Shield
- Motor Servo MG90S (Metal Gear, 4 unit)

## Rangkaian Elektronis

| Servo | Fungsi | Pin Sinyal Arduino |
|---|---|---|
| Servo 1 | Joint 1 (Base) | D4 |
| Servo 2 | Joint 2 (Shoulder) | D5 |
| Servo 3 | Joint 3 (Elbow) | D6 |
| Servo 4 | End Effector | D7 |

Semua servo memakai kabel merah → 5V dan kabel hitam → GND pada Arduino.

## Metode Inverse Kinematics

Panjang lengan: `L1 = 9 cm` (Shoulder–Elbow), `L2 = 8 cm` (Elbow–End Effector).

```
r      = sqrt(x² + y²)
θ1     = atan2(y, x)

a      = sqrt(r² + z²)
α1     = atan2(z, r)
α2     = acos( (L1² + a² − L2²) / (2·L1·a) )
θ2     = α1 + α2

α3     = acos( (L1² + L2² − a²) / (2·L1·L2) )
θ3     = 180° − α3
```

Jika `a > (L1 + L2)`, target berada di luar jangkauan lengan → program menampilkan pesan error.

Karena servo tidak dapat menerima sudut negatif, `θ1` negatif dikonversi dengan
menambahkan 180° (contoh: `-56.31° + 180° = 123.69°`) — ini adalah penyesuaian
kuadran, bukan kesalahan perhitungan.

## Cara Pakai
1. Rangkai hardware sesuai skema di atas dan unggah `src/robot_arm_3dof.ino` ke Arduino Uno R3.
2. Buka Serial Monitor (baud rate `9600`).
3. Masukkan koordinat target dengan format `x,y,z`, misalnya:
   ```
   8,0,9
   ```
4. Robot akan bergerak halus menuju posisi target, menampilkan sudut `θ1, θ2, θ3`,
   lalu menjalankan gerakan end-effector (menyapu 0°→180°→0°).

## Hasil Pengujian (Ringkasan)

10 titik koordinat diuji dan dibandingkan antara **perhitungan teori**, **hasil program**
(serial monitor), dan **implementasi hardware** (diukur memakai kertas kotak-kotak,
1 kotak = 1,25 cm × 1,25 cm, dan busur derajat).

| Perbandingan | Rerata Error θ1 | Rerata Error θ2 | Rerata Error θ3 |
|---|---|---|---|
| Hardware vs Teori | 18,71%* | 0,88% | 2,02% |
| Hardware vs Program | 4,86%* | 0,88% | 2,02% |

\*Lonjakan error θ1 pada titik uji ke-10 (koordinat `x=-2, y=3, z=5`) disebabkan perbedaan
representasi sudut negatif antara teori (`-56,31°`) dan program/hardware (`123,69°` setelah
penyesuaian kuadran), bukan kesalahan sistem. Tanpa titik ini, error θ1 jauh lebih kecil.

Secara umum, error θ2 dan θ3 berada di bawah 5%, membuktikan logika Inverse Kinematics
pada program sudah valid. Selisih kecil yang tersisa disebabkan oleh keterbatasan mekanis
(backlash roda gigi servo) dan resolusi motor servo.

## Kelebihan dan Kekurangan
**Kelebihan:** Robot bekerja baik dengan logika Inverse Kinematic — hasil perhitungan
teori dan sudut aktual lengan memiliki error kecil yang masih dapat ditoleransi.

**Kekurangan:** End-effector hanya dapat berputar (menyapu), belum bisa mengambil/menjepit barang.

## Saran Pengembangan
- Mengganti end-effector dengan gripper/mekanisme penjepit menggunakan servo bertorsi lebih tinggi.
- Menambahkan kontrol *closed-loop* agar pergerakan lebih presisi dan konsisten.
- Mengganti motor servo dengan motor stepper untuk gerakan yang lebih halus dan presisi.

## Dokumentasi Foto Pengujian
Foto pada folder `images/` menunjukkan pengukuran posisi real end-effector (dalam satuan
kotak, 1 kotak = 1,25 cm) untuk memvalidasi hasil terhadap koordinat input:

- `uji_koordinat_1_x8_y0_z9.jpeg` → Input (8, 0, 9)
- `uji_koordinat_2_x6_y2_z12.jpeg` → Input (6, 2, 12)
- `uji_koordinat_3_x2_y3_z5.jpeg` → Input (2, 3, 5)

Video dokumentasi: https://drive.google.com/file/d/14h4C-VRhRyl53dDJ5pozWomYI-9o69QT/view?usp=drivesdk

## Referensi
1. M. Faiq, W. Satriatama, and L. Halim, "Perancangan Awal dan Simulasi Lengan Robot 3 Derajat Kebebasan Berbasis Arduino yang Dikontrol dengan Aplikasi," *J. Mech. Eng. Mechatronics*, vol. 7, no. 2, pp. 118–130, 2022.
2. I. Sulaeman, A. W. Dani, and T. Pangaribowo, "Analisa Inverse Kinematics Pada Prototype 3-DoF Arm Robot Dengan Metode Anfis," *J. Teknol. Elektro*, vol. 13, no. 1, p. 14, 2022.
3. Kurniawan, E. F. (2023). *Rancang Bangun Robot Lengan 3DOF Pengantar Minuman Dengan Metode Inverse Kinematik*. Surabaya: Universitas Dinamika.
4. Ajitesh, K. (2022). *Degree of Freedom in Statistics: Meaning & Examples*.
5. Sinaupedia. (2020). *Pengertian Motor Servo*.
6. Arduino, *Arduino® UNO R3 datasheet*, SKU A000066, 2025.
7. Dodit Suprianto, V.A. (2019). *Microcontroller Arduino untuk Pemula*. Malang: Jasakom.

## Lisensi
Lihat file [LICENSE](LICENSE).
