//            头
//   ----             ----
// | 02/4 |         | 15/8 |
//   ----  --------   ----
//       | 04/2  13/7 |
//       |            |
//       | 05/1  12/6 |
//   ----  --------   ----
// | 16/0 |         | 14/5 |
//   ----             ----

#include <Servo.h>
#include <EEPROM.h>

// Firmware version
String FW_Version = "科睿 - 小蜘蛛 v1.0";

// Servos matrix
const int ALLMATRIX = 9; // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2 + Run Time
const int ALLSERVOS = 8; // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2

// MG90S servo PWM pulse traveling
const int PWMRES_Min = 1; // PWM Resolution 1
const int PWMRES_Max = 180; // PWM Resolution 180
const int SERVOMIN = 400; // 400
const int SERVOMAX = 2400; // 2400

// Servo delay base time
const int BASEDELAYTIME = 10; // 10 ms

// Motion data index
int Servo_PROGRAM;
bool wasHi = 0;

// Servo ID
int GPIO_ID;
int GPIO14_PWM;
int GPIO12_PWM;
int GPIO13_PWM;
int GPIO15_PWM;
int GPIO16_PWM;
int GPIO5_PWM;
int GPIO4_PWM;
int GPIO2_PWM;

// Backup servo value
int Running_Servo_POS [ALLMATRIX];

Servo GPIO14SERVO;
Servo GPIO12SERVO;
Servo GPIO13SERVO;
Servo GPIO15SERVO;
Servo GPIO16SERVO;
Servo GPIO5SERVO;
Servo GPIO4SERVO;
Servo GPIO2SERVO;



// Action
// --------------------------------------------------------------------------------

// Servo zero position 歸零位置
// ----------------------------- G14+,G12+,G13+,G15+, G16+, G5+, G4-, G2-,  ms+
int Servo_Act_0 [ ] PROGMEM = {  60,  120,  45,  150,  140,  45,  160,  40,  500  };

// Start position 起始位置
// ----------------------------- G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
int Servo_Act_1 [ ] PROGMEM = {  60,  120,  45, 150, 140,  45,  160,  40,  500  };;

// Standby 待機
int Servo_Prg_1_Step = 2;
int Servo_Prg_1 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   60, 120, 45, 150, 140,  45,  160,  40,  500  }, // servo center point
  {   40, 120, 45, 170, 160,  45,  160,  20,  500  }, // standby
};

// Forward 前行
int Servo_Prg_2_Step = 11;
int Servo_Prg_2 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // standby 
  {   60,  120,  45,  170,  160,  45,  115,  40,  200  }, // leg1,4 up; leg4 fw
  {   40,  120,  45,  170,  160,  45,  115,  20,  200  }, // leg1,4 dn
  {   40,  120,  45,  150,  140,  45,  115,  20,  200  }, // leg2,3 up
  {   40,   75,  90,  150,  140,  45,  160,  20,  200  }, // leg1,4 bk; leg2 fw
  {   40,   75,  90,  170,  160,  45,  160,  20,  200  }, // leg2,3 dn
  {   60,  120,  90,  170,  160,  45,  160,  40,  200  }, // leg1,4 up; leg1 fw
  {   60,  120,  45,  170,  160,  90,  160,  40,  200  }, // leg2,3 bk
  {   40,  120,  45,  170,  160,  90,  160,  20,  200  }, // leg1,4 dn
  {   40,  120,  45,  170,  140,  90,  160,  20,  200  }, // leg3 up
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // leg3 fw dn
};

// Backward 退後
int Servo_Prg_3_Step = 11;
int Servo_Prg_3 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // standby
  {   60,   75,  45,  170,  160,  45,  160,  40,  200  }, // leg4,1 up; leg1 fw
  {   40,   75,  45,  170,  160,  45,  160,  20,  200  }, // leg4,1 dn
  {   40,   75,  45,  150,  140,  45,  160,  20,  200  }, // leg3,2 up
  {   40,  120,  45,  150,  140,  90,  115,  20,  200  }, // leg4,1 bk; leg3 fw
  {   40,  120,  45,  170,  160,  90,  115,  20,  200  }, // leg3,2 dn
  {   60,  120,  45,  170,  160,  90,  160,  40,  200  }, // leg4,1 up; leg4 fw
  {   60,  120,  90,  170,  160,  45,  160,  40,  200  }, // leg3,1 bk
  {   40,  120,  90,  170,  160,  45,  160,  20,  200  }, // leg4,1 dn
  {   40,  120,  90,  150,  160,  45,  160,  20,  200  }, // leg2 up
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // leg2 fw dn
};

// Left shift 左移
int Servo_Prg_4_Step = 11;
int Servo_Prg_4 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15,  G16,  G5,  G4,  G2,  ms
  {   40, 120,  45, 170,  160,  45,  160,  20,  200  }, // standby
  {   40, 120,   0, 150,  140,  45,  160,  20,  200  }, // leg3,2 up; leg2 fw
  {   40, 120,   0, 170,  160,  45,  160,  20,  200  }, // leg3,2 dn
  {   60, 120,   0, 170,  160,  45,  160,  40,  200  }, // leg1,4 up
  {   60, 165,  45, 170,  160,   0,  160,  40,  200  }, // leg3,2 bk; leg1 fw
  {   40, 165,  45, 170,  160,   0,  160,  20,  200  }, // leg1,4 dn
  {   40, 165,  45, 150,  140,  45,  160,  20,  200  }, // leg3,2 up; leg3 fw
  {   40, 120,  45, 150,  140,  45,  210,  20,  200  }, // leg1,4 bk
  {   40, 120,  45, 170,  160,  45,  210,  20,  200  }, // leg3,2 dn
  {   40, 120,  45, 170,  160,  45,  210,  40,  200  }, // leg4 up
  {   40, 120,  45, 170,  160,  45,  160,  20,  200  }, // leg4 fw dn
};

// Right shift 右移
int Servo_Prg_5_Step = 11;
int Servo_Prg_5 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // standby
  {   40,  120,  45,  150,  140,   0,  160,  20,  200  }, // leg2,3 up; leg3 fw
  {   40,  120,  45,  170,  160,   0,  160,  20,  200  }, // leg2,3 dn
  {   60,  120,  45,  170,  160,   0,  160,  40,  200  }, // leg4,1 up
  {   60,  120,   0,  170,  160,  45,  210,  40,  200  }, // leg2,3 bk; leg4 fw
  {   40,  120,   0,  170,  160,  45,  210,  20,  200  }, // leg4,1 dn
  {   40,  120,  45,  150,  140,  45,  210,  20,  200  }, // leg2,3 up; leg2 fw
  {   40,  165,  45,  150,  140,  45,  160,  20,  200  }, // leg4,1 bk
  {   40,  165,  45,  170,  160,  45,  160,  20,  200  }, // leg2,3 dn
  {   60,  165,  45,  170,  160,  45,  160,  20,  200  }, // leg1 up
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // leg1 fw dn
};

// Turn left 左轉leg
int Servo_Prg_6_Step = 8;
int Servo_Prg_6 [][ALLMATRIX] PROGMEM = {
  // G14,  G12,  G13, G15,  G16,  G5,   G4,  G2,  ms
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // standby
  {   60,  120,  45,  170,  160,  45,  160,  40,  200  }, // leg1,4 up
  {   60,  165,  45,  170,  160,  45,  205,  40,  200  }, // leg1,4 turn
  {   40,  165,  45,  170,  160,  45,  205,  20,  200  }, // leg1,4 dn
  {   40,  165,  45,  150,  140,  45,  205,  20,  200  }, // leg2,3 up
  {   40,  165,  90,  150,  140,  90,  205,  20,  200  }, // leg2,3 turn
  {   40,  165,  90,  170,  160,  90,  205,  20,  200  }, // leg2,3 dn
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // leg1,2,3,4 turn
};

// Turn right 右轉
int Servo_Prg_7_Step = 8;
int Servo_Prg_7 [][ALLMATRIX] PROGMEM = {
  // G14,  G12,  G13, G15,  G16,  G5,  G4,   G2,  ms
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // standby 
  {   40,  120,  45,  150,  140,  45,  160,  20,  200  }, // leg2,3 up
  {   40,  120,   0,  150,  140,   0,  160,  20,  200  }, // leg2,3 turn
  {   40,  120,   0,  170,  160,   0,  160,  20,  200  }, // leg2,3 dn
  {   60,  120,   0,  170,  160,   0,  160,  40,  200  }, // leg1,4 up
  {   60,   75,   0,  170,  160,   0,  115,  40,  200  }, // leg1,4 turn
  {   40,   75,   0,  170,  160,   0,  115,  20,  200  }, // leg1,4 dn
  {   40,  120,  45,  170,  160,  45,  160,  20,  200  }, // leg1,2,3,4 turn
};

// Lie 趴地
int Servo_Prg_8_Step = 1;
int Servo_Prg_8 [][ALLMATRIX] PROGMEM = {
  // G14,  G12,  G13, G15,  G16,  G5,  G4,  G2,  ms 
  {  100,  120,  45,  110,  100,  45,  160, 80,  15000  }, // leg1,4 up
};

// Say Hi 打招呼
int Servo_Prg_9_Step = 7;
int Servo_Prg_9 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   40,  120,  45,  150,  140,  45,  160,  40,  400}, // leg2,3,4 dn
  {  140,  120,  45,  150,  140,  45,  160,  40,  400}, // leg1 up
  {  140,  160,  45,  150,  140,  45,  160,  40,  400}, // leg1 left
  {  140,   80,  45,  150,  140,  45,  160,  40,  400}, // leg1 right
  {  140,  160,  45,  150,  140,  45,  160,  40,  400}, // leg1 left
  {  140,  120,  45,  150,  140,  45,  160,  40,  400}, // leg1 right
  {   40,  120,  45,  150,  140,  45,  160,  40,  400}, // leg1 dn
};

// Fighting 戰鬥姿態
int Servo_Prg_10_Step = 11;
int Servo_Prg_10 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms 
  {   90,  120,  45,  170,  110,  45,  160,  20,  500  }, // leg1, 2 down
  {   90,  120,  25,  170,  110,  15,  140,  20,  500  }, // body turn left
  {   90,  140,  65,  170,  110,  65,  180,  20,  500  }, // body turn right
  {   90,  100,  25,  170,  110,  25,  140,  20,  500  }, // body turn left
  {   90,  140,  65,  170,  110,  65,  180,  20,  500  }, // body turn right
  {   40,  120,  45,  130,  160,  45,  160,  60,  500  }, // leg1, 2 up ; leg3, 4 down
  {   40,  100,  25,  130,  160,  25,  140,  60,  500  }, // body turn left
  {   40,  140,  65,  130,  160,  65,  180,  60,  500  }, // body turn right
  {   40,  100,  25,  130,  160,  25,  140,  60,  500  }, // body turn left
  {   40,  140,  65,  130,  160,  65,  180,  60,  500  }, // body turn right
  {   40,  120,  45,  130,  160,  45,  160,  60,  500  }  // leg1, 2 up ; leg3, 4 down
};

// Push up 掌上壓
int Servo_Prg_11_Step = 12;
int Servo_Prg_11 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   40,  120,  45, 170, 160,  45,  160,  20,  500  }, // start
  {   70,  120,  45,  80,  80,  45,  160,  50,  600  }, // down
  {   40,  120,  45, 170, 160,  45,  160,  20,  500  }, // up 
  {   70,  120,  45,  80,  80,  45,  160,  50,  800  }, // down
  {   40,  120,  45, 170, 160,  45,  160,  20,  700  }, // up
  {   70,  120,  45,  80,  80,  45,  160,  50,  1000  }, // down
  {   40,  120,  45, 170, 160,  45,  160,  20,  900  }, // up
  {   40,  120,  45, 170, 160,  45,  160,  20,  300  }, // up
  {  100,  120,  45,  110,  100,  45,  160, 80,  100  }, // fast down !!!!!!!!!!!!!!!!!!!!!!
  {  140,  120,  45,  150,  140,  45,  160,  40,  800}, // leg1 up
  {   40,  120,  90,  150,  160,  45,  160,  20,  600  }, // leg2 up
  {   40,  120,  45, 170, 160,  45,  160,  20,  800  }  // leg3, leg4 up
};

// Sleep 睡眠姿勢
int Servo_Prg_12_Step = 2;
int Servo_Prg_12 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  500  }, // leg1,4 dn
  {   30,  45, 135, 150, 150, 135,  45,  30,  500  }, // protect myself
};

// 舞步 1
int Servo_Prg_13_Step = 10;
int Servo_Prg_13 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   60,  120,  45,  150,  140,  45,  160,  40,  400  }, // leg1,2,3,4 up
  {   20,  120,  45,  150,  140,  45,  160,  40,  400  }, // leg1 dn
  {   60,  120,  45,  190,  140,  45,  160,  40,  400  }, // leg1 up; leg2 dn
  {   60,  120,  45,  150,  140,  45,  160,  40,  400  }, // leg2 up; leg4 dn
  {   60,  120,  45,  150,  180,  45,  160,  40,  400  }, // leg4 up; leg3 dn
  {   20,  120,  45,  150,  140,  45,  160,  40,  400  }, // leg3 up; leg1 dn
  {   60,  120,  45,  190,  140,  45,  160,  40,  400  }, // leg1 up; leg2 dn
  {   60,  120,  45,  150,  140,  45,  160,   0,  400  }, // leg2 up; leg4 dn
  {   60,  120,  45,  150,  180,  45,  160,  40,  400  }, // leg4 up; leg3 dn
  {   60,  120,  45,  150,  140,  45,  160,  40,  400  }, // leg3 up
};

// 舞步 2
int Servo_Prg_14_Step = 10;
int Servo_Prg_14 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   40,  75,  90,  170,  160,  90,  115,  20,  400  }, // leg1,2,3,4 two sides
  {   85,  75,  90,  125,  115,  90,  115,  20,  400  }, // leg1,2 up
  {   40,  75,  90,  170,  160,  90,  115,  65,  400  }, // leg1,2 dn; leg3,4 up
  {   85,  75,  90,  125,  115,  90,  115,  20,  400  }, // leg3,4 dn; leg1,2 up
  {   40,  75,  90,  170,  160,  90,  115,  65,  400  }, // leg1,2 dn; leg3,4 up
  {   85,  75,  90,  125,  115,  90,  115,  20,  400  }, // leg3,4 dn; leg1,2 up
  {   40,  75,  90,  170,  160,  90,  115,  65,  400  }, // leg1,2 dn; leg3,4 up
  {   85,  75,  90,  125,  115,  90,  115,  20,  400  }, // leg3,4 dn; leg1,2 up
  {   45,  75,  90,  165,  155,  90,  115,  20,  400  }, // leg1,2 dn
  {  60,  120,  45,  150,  140,  45,  160,  40,  500  },
};

// 舞步 3
int Servo_Prg_15_Step = 10;
int Servo_Prg_15 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms {  60,  120,  45,  150,  140,  45,  160,  40,  500  };
  {   40,   75,   0,  170,  160,  90,  205,  20,  400  }, // leg1,2,3,4 bk
  {   80,   75,   0,  120,  110,  90,  205,  20,  400  }, // leg1,2,3 up
  {   40,   75,   0,  170,  160,  90,  205,  20,  400  }, // leg1,2,3 dn
  {   80,   75,   0,  170,  160,  90,  205,  70,  400  }, // leg1,3,4 up
  {   40,   75,   0,  170,  160,  90,  205,  20,  400  }, // leg1,3,4 dn
  {   80,   75,   0,  120,  110,  90,  205,  20,  400  }, // leg1,2,3 up
  {   40,   75,   0,  170,  160,  90,  205,  20,  400  }, // leg1,2,3 dn
  {   80,   75,   0,  170,  160,  90,  205,  70,  400  }, // leg1,3,4 up
  {   40,   75,   0,  170,  160,  90,  205,  20,  400  }, // leg1,3,4 dn
  {   40,  120,  45,  170,  160,  45,  160,  20,  400  }, // standby
};

// --------------------------------------------------------------------------------



// Servo
// --------------------------------------------------------------------------------

void Set_PWM_to_Servo(int iServo, int iValue)
{
  // 讀取 EEPROM 修正誤差
  int NewPWM = iValue + (int8_t)EEPROM.read(iServo);

  NewPWM = map(NewPWM, PWMRES_Min, PWMRES_Max, SERVOMIN, SERVOMAX);

  if (iServo >= 7) {
    GPIO2SERVO.write(NewPWM);
  } else if (iServo >= 6) {
    GPIO4SERVO.write(NewPWM);
  } else if (iServo >= 5) {
    GPIO5SERVO.write(NewPWM);
  } else if (iServo >= 4) {
    GPIO16SERVO.write(NewPWM);
  } else if (iServo >= 3) {
    GPIO15SERVO.write(NewPWM);
  } else if (iServo >= 2) {
    GPIO13SERVO.write(NewPWM);
  } else if (iServo >= 1) {
    GPIO12SERVO.write(NewPWM);
  } else if (iServo == 0) {
    GPIO14SERVO.write(NewPWM);
  }
}

void Servo_PROGRAM_Zero()
{
  // 清除備份目前馬達數值
  for (int Index = 0; Index < ALLMATRIX; Index++) {
    Running_Servo_POS[Index] = Servo_Act_0[Index];
  }

  // 重新載入馬達預設數值
  for (int iServo = 0; iServo < ALLSERVOS; iServo++) {
    Set_PWM_to_Servo(iServo, Running_Servo_POS[iServo]);
    delay(10);
  }
}

void Servo_PROGRAM_Center()
{
  // 清除備份目前馬達數值
  for (int Index = 0; Index < ALLMATRIX; Index++) {
    Running_Servo_POS[Index] = Servo_Act_1[Index];
  }

  // 重新載入馬達預設數值
  for (int iServo = 0; iServo < ALLSERVOS; iServo++) {
    Set_PWM_to_Servo(iServo, Running_Servo_POS[iServo]);
    delay(10);
  }
}

void Servo_PROGRAM_Run(int iMatrix[][ALLMATRIX], int iSteps)
{
  int INT_TEMP_A, INT_TEMP_B, INT_TEMP_C;

  for (int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++) { // iSteps 步驟主迴圈

    int InterTotalTime = iMatrix[MainLoopIndex][ALLMATRIX - 1]; // InterTotalTime 此步驟總時間

    int InterDelayCounter = InterTotalTime / BASEDELAYTIME; // InterDelayCounter 此步驟基本延遲次數

    for (int InterStepLoop = 0; InterStepLoop < InterDelayCounter; InterStepLoop++) { // 內差次數迴圈

      for (int ServoIndex = 0; ServoIndex < ALLSERVOS; ServoIndex++) { // 馬達主迴圈

        INT_TEMP_A = Running_Servo_POS[ServoIndex]; // 馬達現在位置
        INT_TEMP_B = iMatrix[MainLoopIndex][ServoIndex]; // 馬達目標位置

        if (INT_TEMP_A == INT_TEMP_B) { // 馬達數值不變
          INT_TEMP_C = INT_TEMP_B;
        } else if (INT_TEMP_A > INT_TEMP_B) { // 馬達數值減少
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_A - INT_TEMP_B); // PWM內差值 = map(執行次數時間累加, 開始時間, 結束時間, 內差起始值, 內差最大值)
          if (INT_TEMP_A - INT_TEMP_C >= INT_TEMP_B) {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A - INT_TEMP_C);
          }
        } else if (INT_TEMP_A < INT_TEMP_B) { // 馬達數值增加
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_B - INT_TEMP_A); // PWM內差值 = map(執行次數時間累加, 開始時間, 結束時間, 內差起始值, 內差最大值)
          if (INT_TEMP_A + INT_TEMP_C <= INT_TEMP_B) {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A + INT_TEMP_C);
          }
        }

      }

      delay(BASEDELAYTIME);
    }

    // 備份目前馬達數值
    for (int Index = 0; Index < ALLMATRIX; Index++) {
      Running_Servo_POS[Index] = iMatrix[MainLoopIndex][Index];
    }
  }
}

void writeKeyValue(int8_t key, int8_t value)
{
  EEPROM.write(key, value);
  EEPROM.commit();
}

int8_t readKeyValue(int8_t key)
{
  Serial.println("read");
  Serial.println(key);

  int8_t value = EEPROM.read(key);
}

// --------------------------------------------------------------------------------


// Setup
// --------------------------------------------------------------------------------

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Q1 mini Start!");

  // Software PWM PIN
  GPIO14SERVO.attach(15, SERVOMIN, SERVOMAX);
  GPIO12SERVO.attach(13, SERVOMIN, SERVOMAX);
  GPIO13SERVO.attach(12, SERVOMIN, SERVOMAX);
  GPIO15SERVO.attach(14, SERVOMIN, SERVOMAX);
  GPIO16SERVO.attach(2, SERVOMIN, SERVOMAX);
  GPIO5SERVO.attach(4, SERVOMIN, SERVOMAX);
  GPIO4SERVO.attach(5, SERVOMIN, SERVOMAX);
  GPIO2SERVO.attach(16, SERVOMIN, SERVOMAX);
  
  // EEPROM
  EEPROM.begin(512);
  delay(10);

  for (int Index = 0; Index < ALLMATRIX; Index++) {
    Running_Servo_POS[Index] = Servo_Act_0[Index];
  }

  Servo_PROGRAM_Zero();
}

// --------------------------------------------------------------------------------



// Loop
// --------------------------------------------------------------------------------

void loop(void)
{
  //тут мод Олега
  if (wasHi == 0)
  {
    Servo_PROGRAM = 9;
    Serial.print((String) "Started with 9\n");
    wasHi = 1;
  }
  else
  {
    Servo_PROGRAM = random(1, 16);
    Serial.print((String) "Action " + Servo_PROGRAM + "\n");
  }
  if (Servo_PROGRAM >= 1 ) {
    switch (Servo_PROGRAM) {
      case 1: // Standby 待機
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        break;
      case 2: // Forward 前行
        Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step);
        break;
      case 3: // Backward 退後
        Servo_PROGRAM_Run(Servo_Prg_3, Servo_Prg_3_Step);
        break;
      case 4: // Left shift 左移
        Servo_PROGRAM_Run(Servo_Prg_4, Servo_Prg_4_Step);
        break;
      case 5: // Right shift 右移
        Servo_PROGRAM_Run(Servo_Prg_5, Servo_Prg_5_Step);
        break;
      case 6: // Turn left 左轉
        Servo_PROGRAM_Run(Servo_Prg_6, Servo_Prg_6_Step);
        break;
      case 7: // Turn right 右轉
        Servo_PROGRAM_Run(Servo_Prg_7, Servo_Prg_7_Step);
        break;
      case 8: // Lie 趴地
        Servo_PROGRAM_Run(Servo_Prg_8, Servo_Prg_8_Step);
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step);
        Servo_PROGRAM_Run(Servo_Prg_6, Servo_Prg_6_Step);
        Servo_PROGRAM_Run(Servo_Prg_11, Servo_Prg_11_Step);
        break;
      case 9: // Say Hi 打招呼
        Servo_PROGRAM_Run(Servo_Prg_9, Servo_Prg_9_Step);
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        break;
      case 10: // Fighting 戰鬥姿態
        Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step);
        Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step);
        Servo_PROGRAM_Run(Servo_Prg_10, Servo_Prg_10_Step);
        break;
      case 11: // Push up 掌上壓
        Servo_PROGRAM_Run(Servo_Prg_11, Servo_Prg_11_Step);
        break;
      case 12: // Sleep 睡眠姿勢 
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        Servo_PROGRAM_Run(Servo_Prg_12, Servo_Prg_12_Step);
        break;
      case 13: // 舞步 1
        Servo_PROGRAM_Run(Servo_Prg_13, Servo_Prg_13_Step);
        break;
      case 14: // 舞步 2
        Servo_PROGRAM_Run(Servo_Prg_14, Servo_Prg_14_Step);
        break;
      case 15: // 舞步 3
        Servo_PROGRAM_Run(Servo_Prg_15, Servo_Prg_15_Step);
        break;
      case 99: // 待機
        Servo_PROGRAM_Center();
        delay(300);
        break;
      case 100: // 歸零姿勢
        Servo_PROGRAM_Zero();
        delay(300);
        break;
    }
    Servo_PROGRAM = 0;
  }
}

// --------------------------------------------------------------------------------
