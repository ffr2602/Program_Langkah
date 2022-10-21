#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)  // When using DynamixelShield
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8);  // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL Serial
#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = 2;      // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE)  // When using DynamixelShield
#define DXL_SERIAL Serial
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO)  // When using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904)  // When using official ROBOTIS board with DXL circuit.
#define DXL_SERIAL Serial3        //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 22;  //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR)  // When using official ROBOTIS board with DXL circuit.
// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84;  // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
//OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
#else  // Other boards when using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#endif

#define BROADCAST_ID 254

const uint8_t Coxa[6] = { 2, 5, 8, 11, 14, 17 };
const uint8_t Femur[6] = { 1, 4, 7, 10, 13, 16 };
const uint8_t Tibia[6] = { 0, 3, 6, 9, 12, 15 };
const float DXL_PROTOCOL_VERSION = 1.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

//Gerakan robot
int posisi, lastPosisi;
int bantere = 500;  //290 340
int langkah = 0;
int ganti;
boolean jalan;
float naik = 0.00;
float napak = 40.00;
float njengat = 70.00;

//Pin komunikasi
int Rx4 = 16;
int Rx3 = 17;
int Rx2 = 18;
int Rx1 = 19;

// Variabel nilai invers setiap kaki
float y04, y01, y02, y03;
float z04, z01, z02, z03;
float x04, x01, x02, x03;
float hadap0, hadap1, hadap2, hadap3;

// Invers
float radian = 0.0174533;
float femurLength = 45.0000000;
float tibiaLength = 73.3000000;

float legLength;
float Z_Length;
float Sud_A, Sud_B, Sud_C;
float Alpha, Betha, Gamma;
int SudutCoxa, SudutFemur, SudutTibia;
float te, ka, ki;
int step = 0;

float lurus = 50.00;

// Nilai Posisi Servo
int ki_deC, ki_deF, ki_deT;
int ki_teC, ki_teF, ki_teT;
int ki_beC, ki_beF, ki_beT;
int ka_deC, ka_deF, ka_deT;
int ka_teC, ka_teF, ka_teT;
int ka_beC, ka_beF, ka_beT;

void setup() {
  DEBUG_SERIAL.begin(57600);
  while (!DEBUG_SERIAL)
    ;

  dxl.begin(115200);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  for (int i = 0; i < 6; ++i) {
    dxl.ping(Coxa[i]);
    dxl.ping(Femur[i]);
    dxl.ping(Tibia[i]);
  }

  for (int i = 0; i < 6; ++i) {
    dxl.torqueOff(Coxa[i]);
    dxl.torqueOff(Femur[i]);
    dxl.torqueOff(Tibia[i]);
    dxl.setOperatingMode(Coxa[i], OP_POSITION);
    dxl.setOperatingMode(Femur[i], OP_POSITION);
    dxl.setOperatingMode(Tibia[i], OP_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  for (int i = 0; i < 6; ++i) {
    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, Coxa[i], 30);
    dxl.writeControlTableItem(PROFILE_VELOCITY, Femur[i], 30);
    dxl.writeControlTableItem(PROFILE_VELOCITY, Tibia[i], 30);
  }
  pinMode(Rx1, INPUT);
  pinMode(Rx2, INPUT);
  pinMode(Rx3, INPUT);
  pinMode(Rx4, INPUT);
  delay(500);
}


void loop() {
  //putarKanan(30.00, 30.00, 30.00);
  //putarKiri(20.00, 20.00, 20.00);
  maju(60.00, 60.00, 60.00); // kiri
  //kemanaKita();
  //langkahTinggi(60.00, 60.00, 60.00);
  //geserKanan(90.00, 90.00, 90.00);
  //mundur(50.00, 50.00, 50.00);
  //siap();
  delay(95);
}

/****************************************************************
           Sub Program
 *****************************************************************/

void kemanaKita() {
  if ((digitalRead(Rx1) == LOW) && (digitalRead(Rx2) == LOW) && (digitalRead(Rx3) == LOW) && (digitalRead(Rx4) == LOW)) {
    siap();
    //SerialUSB.println("siap");
  }
  if ((digitalRead(Rx1) == LOW) && (digitalRead(Rx2) == LOW) && (digitalRead(Rx3) == LOW) && (digitalRead(Rx4) == HIGH)) {
    maju(50.00, 50.00, 50.00);
    //SerialUSB.println("maju");
  }
  if ((digitalRead(Rx1) == LOW) && (digitalRead(Rx2) == LOW) && (digitalRead(Rx3) == HIGH) && (digitalRead(Rx4) == LOW)) {
    mundur(50.00, 50.00, 50.00);
    //SerialUSB.println("mundur");
  }
  if ((digitalRead(Rx1) == LOW) && (digitalRead(Rx2) == LOW) && (digitalRead(Rx3) == HIGH) && (digitalRead(Rx4) == HIGH)) {
    putarKanan(20.00, 20.00, 20.00);
    //SerialUSB.println("putar kanan");
  }
  if ((digitalRead(Rx1) == LOW) && (digitalRead(Rx2) == HIGH) && (digitalRead(Rx3) == LOW) && (digitalRead(Rx4) == LOW)) {
    putarKiri(20.00, 20.00, 20.00);
    //SerialUSB.println("putar kiri");
  }
  if ((digitalRead(Rx1) == LOW) && (digitalRead(Rx2) == HIGH) && (digitalRead(Rx3) == LOW) && (digitalRead(Rx4) == HIGH)) {
    maju(50.00, 25.00, 50.00);
    //SerialUSB.println("serong kanan");
  }
  if ((digitalRead(Rx1) == LOW) && (digitalRead(Rx2) == HIGH) && (digitalRead(Rx3) == HIGH) && (digitalRead(Rx4) == LOW)) {
    maju(50.00, 50.00, 25.00);
    //SerialUSB.println("serong kiri");
  }
  if ((digitalRead(Rx1) == LOW) && (digitalRead(Rx2) == HIGH) && (digitalRead(Rx3) == HIGH) && (digitalRead(Rx4) == HIGH)) {
    maju(50.00, 0.00, 50.00);
    //SerialUSB.println("serong kanan bgt");
  }
  if ((digitalRead(Rx1) == HIGH) && (digitalRead(Rx2) == LOW) && (digitalRead(Rx3) == LOW) && (digitalRead(Rx4) == LOW)) {
    maju(50.00, 50.00, 0.00);
    //SerialUSB.println("serong kiri banget");
  }
  if ((digitalRead(Rx1) == HIGH) && (digitalRead(Rx2) == LOW) && (digitalRead(Rx3) == LOW) && (digitalRead(Rx4) == HIGH)) {
    langkahTinggi(40.00, 40.00, 40.00);
    //SerialUSB.println("langkah tinggi");
  }
  if ((digitalRead(Rx1) == HIGH) && (digitalRead(Rx2) == LOW) && (digitalRead(Rx3) == HIGH) && (digitalRead(Rx4) == LOW)) {
    putarKananT(20.00, 20.00, 20.00);
    //SerialUSB.println("langkah tinggi putar kanan");
  }
  if ((digitalRead(Rx1) == HIGH) && (digitalRead(Rx2) == LOW) && (digitalRead(Rx3) == HIGH) && (digitalRead(Rx4) == HIGH)) {
    putarKiriT(20.00, 20.00, 20.00);
    //SerialUSB.println("langkah tinggi putar kiri");
  }
  if ((digitalRead(Rx1) == HIGH) && (digitalRead(Rx2) == HIGH) && (digitalRead(Rx3) == LOW) && (digitalRead(Rx4) == LOW)) {
    langkahTinggi(50.00, 10.00, 50.00);
    //    geserKanan(70.00, 70.00, 70.00);
    //SerialUSB.println("langkah tinggi serong kanan bgt");
  }
  if ((digitalRead(Rx1) == HIGH) && (digitalRead(Rx2) == HIGH) && (digitalRead(Rx3) == LOW) && (digitalRead(Rx4) == HIGH)) {
    langkahTinggi(50.00, 50.00, 10.00);
    //    geserKiri(70.00, 70.00, 70.00);
    //SerialUSB.println("langkah tinggi serong kiri bgt");
  }
  if ((digitalRead(Rx1) == HIGH) && (digitalRead(Rx2) == HIGH) && (digitalRead(Rx3) == HIGH) && (digitalRead(Rx4) == LOW)) {
    //    geserKanan(70.00, 70.00, 70.00);
    //SerialUSB.println("geser kanan");
  }
}

// .......................... invers ..........................
void invers(float x, float y, float z) {
  legLength = sqrt((x * x) + (y * y));
  Z_Length = sqrt((z * z) + (legLength * legLength));
  Sud_A = atan(legLength / z) / radian;
  Sud_B = acos(((femurLength * femurLength) + (Z_Length * Z_Length) - (tibiaLength * tibiaLength)) / (2 * femurLength * Z_Length)) / radian;
  Sud_C = acos(((femurLength * femurLength) + (tibiaLength * tibiaLength) - (Z_Length * Z_Length)) / (2 * femurLength * tibiaLength)) / radian;

  Alpha = atan(y / x) / radian;
  Betha = (Sud_A + Sud_B) - 90;
  Gamma = 90 - Sud_C;
}

// .................. konversi sudut ke posisi ...................
unsigned int convert(int sudut) {
  return map(sudut, -180, 180, 0, 1023);
}

// ...................... gerakin servo ......................
void gerak() {
  dxl.setGoalPosition(Tibia[1], ka_deT);
  dxl.setGoalPosition(Femur[1], ka_deF);
  dxl.setGoalPosition(Coxa[1], ka_deC);
  
  dxl.setGoalPosition(Tibia[2], ka_teT);
  dxl.setGoalPosition(Femur[2], ka_teF);
  dxl.setGoalPosition(Coxa[2], ka_teC);

  dxl.setGoalPosition(Tibia[5], ka_beT);
  dxl.setGoalPosition(Femur[5], ka_beF);
  dxl.setGoalPosition(Coxa[5], ka_beC);

  dxl.setGoalPosition(Tibia[0], ki_deT);
  dxl.setGoalPosition(Femur[0], ki_deF);
  dxl.setGoalPosition(Coxa[0], ki_deC);

  dxl.setGoalPosition(Tibia[3], ki_teT);
  dxl.setGoalPosition(Femur[3], ki_teF);
  dxl.setGoalPosition(Coxa[3], ki_teC);

  dxl.setGoalPosition(Tibia[4], ki_beT);
  dxl.setGoalPosition(Femur[4], ki_beF);
  dxl.setGoalPosition(Coxa[4], ki_beC);
}

// .......................... velocity ..........................
void velocity(int dir, float Z1, float Z2, float Y1, float Y2, float Y3, float Y4) {
  if (dir == 0) {
    for (char ul = 0; ul < 2; ul++) {
      switch (ul) {
        case 0:
          while (1) {
            if (y01 < Y1) y01 = y01 + 5.00;
            if (y01 > Y1) y01 = y01 - 5.00;
            if (y03 < Y3) y03 = y03 + 5.00;
            if (y03 > Y3) y03 = y03 - 5.00;
            if (z01 < Z1) z01 = z01 + 5.00;
            if (z01 > Z1) z01 = z01 - 5.00;
            invers(50.00, y01, z01);
            ki_teC = convert(Alpha);
            ki_teF = convert(Betha);
            ki_teT = convert(Gamma);
            invers(50.00, y03, z01);
            ka_deC = convert((-1 * Alpha));
            ka_deF = convert(Betha);
            ka_deT = convert(Gamma);
            ka_beC = convert((-1 * Alpha));
            ka_beF = convert(Betha);
            ka_beT = convert(Gamma);
            if ((y01 == Y1) && (y03 == Y3) && (z01 == Z1)) break;
          }
          break;

        case 1:
          while (1) {
            if (y02 < Y2) y02 = y02 + 5.00;
            if (y02 > Y2) y02 = y02 - 5.00;
            if (y04 < Y4) y04 = y04 + 5.00;
            if (y04 > Y4) y04 = y04 - 5.00;
            if (z02 < Z2) z02 = z02 + 5.00;
            if (z02 > Z2) z02 = z02 - 5.00;
            invers(50.00, y02, z02);
            ka_teC = convert((-1 * Alpha));
            ka_teF = convert(Betha);
            ka_teT = convert(Gamma);
            invers(50.00, y04, z02);
            ki_deC = convert(Alpha);
            ki_deF = convert(Betha);
            ki_deT = convert(Gamma);
            ki_beC = convert(Alpha);
            ki_beF = convert(Betha);
            ki_beT = convert(Gamma);
            if ((y02 == Y2) && (y04 == Y4) && (z02 == Z2)) break;
          }
          break;
      }
    }
  }

  if (dir == 1) {
    for (char ul = 0; ul < 2; ul++) {
      switch (ul) {
        case 0:
          while (1) {
            if (y01 < Y1) y01 = y01 + 5.00;
            if (y01 > Y1) y01 = y01 - 5.00;
            if (y03 < Y3) y03 = y03 + 5.00;
            if (y03 > Y3) y03 = y03 - 5.00;
            if (z01 < Z1) z01 = z01 + 5.00;
            if (z01 > Z1) z01 = z01 - 5.00;
            invers(50.00, y01, z01);
            ki_teC = convert(Alpha);
            ki_teF = convert(Betha);
            ki_teT = convert(Gamma);
            invers(50.00, y03, z01);
            ka_deC = convert(Alpha);
            ka_deF = convert(Betha);
            ka_deT = convert(Gamma);
            ka_beC = convert(Alpha);
            ka_beF = convert(Betha);
            ka_beT = convert(Gamma);
            if ((y01 == Y1) && (y03 == Y3) && (z01 == Z1)) break;
          }
          break;

        case 1:
          while (1) {
            if (y02 < Y2) y02 = y02 + 5.00;
            if (y02 > Y2) y02 = y02 - 5.00;
            if (y04 < Y4) y04 = y04 + 5.00;
            if (y04 > Y4) y04 = y04 - 5.00;
            if (z02 < Z2) z02 = z02 + 5.00;
            if (z02 > Z2) z02 = z02 - 5.00;
            invers(50.00, y02, z02);
            ka_teC = convert((-1 * Alpha));
            ka_teF = convert(Betha);
            ka_teT = convert(Gamma);
            invers(50.00, y04, z02);
            ki_deC = convert((-1 * Alpha));
            ki_deF = convert(Betha);
            ki_deT = convert(Gamma);
            ki_beC = convert((-1 * Alpha));
            ki_beF = convert(Betha);
            ki_beT = convert(Gamma);
            if ((y02 == Y2) && (y04 == Y4) && (z02 == Z2)) break;
          }
          break;
      }
    }
  }

  // eksekusi gerak servo
  gerak();
}

void velocityX(float Z1, float Z2, float X1, float X2, float X3, float X4) {
  for (char ul = 0; ul < 2; ul++) {
    switch (ul) {
      case 0:
        while (1) {
          if (x01 < X1) x01 = x01 + 5.00;
          if (x01 > X1) x01 = x01 - 5.00;
          if (x03 < X3) x03 = x03 + 5.00;
          if (x03 > X3) x03 = x03 - 5.00;
          if (z01 < Z1) z01 = z01 + 5.00;
          if (z01 > Z1) z01 = z01 - 5.00;
          invers(x01, 0.00, z01);
          ki_teC = convert(Alpha);
          ki_teF = convert(Betha);
          ki_teT = convert(Gamma);
          invers(x03, 0.00, z01);
          ka_deC = convert((-1 * Alpha));
          ka_deF = convert(Betha);
          ka_deT = convert(Gamma);
          ka_beC = convert((-1 * Alpha));
          ka_beF = convert(Betha);
          ka_beT = convert(Gamma);
          if ((x01 == X1) && (x03 == X3) && (z01 == Z1)) break;
        }
        break;

      case 1:
        while (1) {
          if (x02 < X2) x02 = x02 + 5.00;
          if (x02 > X2) x02 = x02 - 5.00;
          if (x04 < X4) x04 = x04 + 5.00;
          if (x04 > X4) x04 = x04 - 5.00;
          if (z02 < Z2) z02 = z02 + 5.00;
          if (z02 > Z2) z02 = z02 - 5.00;
          invers(x02, 0.00, z02);
          ka_teC = convert((-1 * Alpha));
          ka_teF = convert(Betha);
          ka_teT = convert(Gamma);
          invers(x04, 0.00, z02);
          ki_deC = convert(Alpha);
          ki_deF = convert(Betha);
          ki_deT = convert(Gamma);
          ki_beC = convert(Alpha);
          ki_beF = convert(Betha);
          ki_beT = convert(Gamma);
          if ((x02 == X2) && (x04 == X4) && (z02 == Z2)) break;
        }
        break;
    }
  }

  // eksekusi gerak servo
  gerak();
}

// ================================= Gerakan Dasar =================================

void siap() {
  jalan = false;
  velocity(0, 40.00, 40.00, 0.00, 0.00, 0.00, 0.00);
}

void maju(float _te, float _ka, float _ki) {
  te = _te;
  ka = _ka;
  ki = _ki;
  if (step == 0) { velocity(0, naik, napak, 0, 0, 0, 0); }
  if (step == 1) { velocity(0, napak, napak, te, -te, ka, -ki); }
  if (step == 2) { velocity(0, napak, naik, 0, 0, 0, 0); }
  if (step == 3) { velocity(0, napak, napak, -te, te, -ka, ki); }
  step = step + 1;
  if (step == 4) step = 0;
}

void mundur(float _te, float _ka, float _ki) {
  te = _te;
  ka = _ka;
  ki = _ki;
  if (step == 2) { velocity(0, naik, napak, 0, 0, 0, 0); }
  if (step == 1) { velocity(0, napak, napak, te, -te, ka, -ki); }
  if (step == 0) { velocity(0, napak, naik, 0, 0, 0, 0); }
  if (step == 3) { velocity(0, napak, napak, -te, te, -ka, ki); }
  step = step + 1;
  if (step == 4) step = 0;
}

void putarKiri(float _te, float _ka, float _ki) {
  te = _te;
  ka = _ka;
  ki = _ki;
  if (step == 0) { velocity(1, naik, napak, 0, 0, 0, 0); }
  if (step == 1) { velocity(1, napak, napak, -te, -te, -ka, -ki); }
  if (step == 2) { velocity(1, napak, naik, 0, 0, 0, 0); }
  if (step == 3) { velocity(1, napak, napak, te, te, ka, ki); }
  step = step + 1;
  if (step == 4) step = 0;
}

void putarKanan(float _te, float _ka, float _ki) {
  te = _te;
  ka = _ka;
  ki = _ki;
  if (step == 0) { velocity(1, napak, naik, 0, 0, 0, 0); }
  if (step == 1) { velocity(1, napak, napak, -te, -te, -ka, -ki); }
  if (step == 2) { velocity(1, naik, napak, 0, 0, 0, 0); }
  if (step == 3) { velocity(1, napak, napak, te, te, ka, ki); }
  step = step + 1;
  if (step == 4) step = 0;
}

void langkahTinggi(float _te, float _ka, float _ki) {
  te = _te;
  ka = _ka;
  ki = _ki;
  if (step == 2) { velocity(0, naik, njengat, 0, 0, 0, 0); }
  if (step == 1) { velocity(0, njengat, njengat, te, -te, ka, -ki); }
  if (step == 0) { velocity(0, njengat, naik, 0, 0, 0, 0); }
  if (step == 3) { velocity(0, njengat, njengat, -te, te, -ka, ki); }
  step = step + 1;
  if (step == 4) step = 0;
  delay(500);
}

void geserKanan(float _te, float _ka, float _ki) {
  te = _te;
  ka = _ka;
  ki = _ki;
  if (step == 0) { velocityX(naik, napak, 50.00, 50.00, 50.00, 50.00); }
  if (step == 1) { velocityX(napak, napak, te, -te, ka, -ki); }
  if (step == 2) { velocityX(napak, naik, 50.00, 50.00, 50.00, 50.00); }
  if (step == 3) { velocityX(napak, napak, -te, te, -ka, ki); }
  step = step + 1;
  if (step == 4) step = 0;
  delay(100);
}

void geserKiri(float _te, float _ka, float _ki) {
  te = _te;
  ka = _ka;
  ki = _ki;
  if (step == 2) { velocityX(naik, napak, 50.00, 50.00, 50.00, 50.00); }
  if (step == 1) { velocityX(napak, napak, te, -te, ka, -ki); }
  if (step == 0) { velocityX(napak, naik, 50.00, 50.00, 50.00, 50.00); }
  if (step == 3) { velocityX(napak, napak, -te, te, -ka, ki); }
  step = step + 1;
  if (step == 4) step = 0;
}

void putarKiriT(float _te, float _ka, float _ki) {
  te = _te;
  ka = _ka;
  ki = _ki;
  if (step == 0) { velocity(1, naik, njengat, 0, 0, 0, 0); }
  if (step == 1) { velocity(1, njengat, njengat, -te, -te, -ka, -ki); }
  if (step == 2) { velocity(1, njengat, naik, 0, 0, 0, 0); }
  if (step == 3) { velocity(1, njengat, njengat, te, te, ka, ki); }
  step = step + 1;
  if (step == 4) step = 0;
  delay(500);
}

void putarKananT(float _te, float _ka, float _ki) {
  te = _te;
  ka = _ka;
  ki = _ki;
  if (step == 0) { velocity(1, njengat, naik, 0, 0, 0, 0); }
  if (step == 1) { velocity(1, njengat, njengat, -te, -te, -ka, -ki); }
  if (step == 2) { velocity(1, naik, njengat, 0, 0, 0, 0); }
  if (step == 3) { velocity(1, njengat, njengat, te, te, ka, ki); }
  step = step + 1;
  if (step == 4) step = 0;
  delay(500);
}