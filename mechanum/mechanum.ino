/************************
 * 2024年度 元木大輔
 * 卒検のメカナムロボのプログラム
 * 4つのMD
 * エンコーダ
 * 
 * 
 *************************/
#include <Wire.h>
 
//MD pin
#define AF 22//F...front, B...back 
#define AB 24
#define APWM 2
#define BF 26
#define BB 28
#define BPWM 3
#define CF 36
#define CB 34
#define CPWM 5
#define DF 32
#define DB 30
#define DPWM 4

/*

メカナムホイールの配置
A||---||D
  |   |
B||---||C




 */

//encoder pin
#define A_AP A15 //A_AP...Aport_A_Phase
#define A_BP 2
#define B_AP A13
#define B_BP A12
#define C_AP A9
#define C_BP A8
#define D_AP A11
#define D_BP A10

#define GEAR 35 //gear ratio ギア比
#define PPR 11 //encoder PPR

//robot size
#define lx 0.047 //実測値[m]　誤差すごいかも
#define ly 0.1045 //寸法図から計算
#define wheel_r 0.04 //radius

#define wheel_k 0.011435 //車輪の回転数から速度を出す係数k
/*
 *弧の長さl
 *l = 2*π*(wheel_r)*(count/1540) countはエンコーダのパルス数
 * 2*π*0.04/1540 = 1.63199618...*10^-4
 * l = 0.00001632*count [m]
 * 速さvは
 * v = l/t (tは割り込みの計算周期0.1[s]
 *   = (0.0001632/0.1)*count
 *   = 0.001632*count　[m/s]
 * v = l/t (tは割り込みの計算周期0.014272[s]
 *   = (0.0001632/0.014272)*count
 *   = 0.01143497758*count　[m/s]
 */

const double res = GEAR * PPR * 4; //encoder resolution

//GYT521 (MPU6050)

#define ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define CONFIG 0x1A
#define ACCEL_CONFIG 0x1C

double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ, avAcX = 0;
double rad = 0;
double x = 0, y = 0, deg = 0;
double slptime = 0.014272 * 7;

//filter
double fc = 0.6; //filter coefficient ローパスフィルタの係数　1に近づけるほど平滑化の度合いが大きい
double lpv = 0; //low-pass filter value
double hpv = 0; //high-pass filter value
double Acc = 0, Sp = 0; //加速度，速度
double oldAcc = 0, oldSp = 0; //一つ前の加速度，速度
bool sflg = true; //trueでロボット停止してる

void setup() {
  
  Serial.begin(115200);

  //MD setting
  pinMode(AF, OUTPUT);
  pinMode(AB, OUTPUT);
  pinMode(APWM, OUTPUT);
  
  pinMode(BF, OUTPUT);
  pinMode(BB, OUTPUT);
  pinMode(BPWM, OUTPUT);
  
  pinMode(CF, OUTPUT);
  pinMode(CB, OUTPUT);
  pinMode(CPWM, OUTPUT);
  
  pinMode(DF, OUTPUT);
  pinMode(DB, OUTPUT);
  pinMode(DPWM, OUTPUT);

  //encoder setting
  pinMode(A_AP, INPUT);
  pinMode(A_BP, INPUT);
  pinMode(B_AP, INPUT);
  pinMode(B_BP, INPUT);
  pinMode(C_AP, INPUT);
  pinMode(C_BP, INPUT);
  pinMode(D_AP, INPUT);
  pinMode(D_BP, INPUT);

  //debug pin
  pinMode(53, OUTPUT);
  //change interrupt (for encoder)
  PCMSK2 = 0b11111111; //pin change msk (PCINT23-16)
  PCICR = 0b00000100;  //use INT2 

  //timer interrupt
//  Serial.println(TCCR2A); //config
//  Serial.println(TCCR2B); //config
//  Serial.println(TCNT2);
//  Serial.println(OCR2A); //compare match A
//  Serial.println(OCR2B); //compare match B
//  Serial.println(TIMSK2);
//  Serial.println(TIFR2);
  TCCR2A = 0b00000010; //CTC mode
  TCCR2B = 0b00000111; //2-0bit is prescaler setting 111...1:1024
  OCR2B = 0;
  OCR2A = 222; 
  /*
   * ((pre)*(OCR2A+1))/Fosc)
   * (1024 * (222+1)) / (16*10^-6) = 0.014272 [ms/interrupt]
   */
  TIMSK2 = 0b00000010; //compare match A interrupt enable

  //i2c GYT521
  Wire.begin();
  Wire.beginTransmission(ADDR);
  Wire.write(PWR_MGMT_1);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(ADDR);
  Wire.write(CONFIG);  // PWR_MGMT_1 register
  Wire.write(6);     // set to DigitalLowPassFilter 4:BandWidth:21[Hz], Delay:8.5[ms](Gyro:20[Hzz], 8.3[ms])
  Wire.endTransmission(true);
  Wire.beginTransmission(ADDR);
  Wire.write(ACCEL_CONFIG);  // PWR_MGMT_1 register
  Wire.write(0b00001000);     // FullScaleRange:+-4g (LSB Sensitivity:8192 LSB/g)
  Wire.endTransmission(true);
  delay(1000);
  for(int i = 0; i < 10; i++){
    Wire.beginTransmission(ADDR);
    Wire.write(ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR,14,true);  // request a total of 14 registers
    AcX=(int16_t)Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    avAcX += AcX / 10;
    delay(100);
    //Serial.println(avAcX); //debug
  }
  
}

char rcv = 'l';
unsigned char pwm = 0;
unsigned char pwm_ary[4] = {0, 0, 0, 0}; //A, B, C, D
unsigned char pwm_pin[4] = {APWM, BPWM, CPWM, DPWM};
int count[4] = {0,0,0,0}; //encoder pulse counter
unsigned char old = 0;
unsigned char buf; 
bool flag = false;

double xyt[3] = {0,0,0};

void loop() {
  if(Serial.available()>0){
    rcv = Serial.read();
    if(rcv == 'w'){
      //Serial.println("前進");
      north();
    }else if(rcv == 'a'){
      //Serial.println("左");
      west();
    }else if(rcv == 's'){
      //Serial.println("後退");
      south();
    }else if(rcv == 'd'){
      //Serial.println("右");
      east();
    }else if(rcv == 'q'){
      //Serial.println("左前");
      northwest();
    }else if(rcv == 'e'){
      //Serial.println("右前");
      northeast();
    }else if(rcv == 'c'){
      //Serial.println("右後ろ");
      southeast();
    }else if(rcv == 'z'){
      //Serial.println("左後ろ");
      southwest();
    }else if(rcv == 'f'){
      //Serial.println("左回転");
      leftrotate();
    }else if(rcv == 'g'){
      //Serial.println("右回転");
      rightrotate();
    }else if(rcv == 'x'){
      //Serial.println("停止");
      motorstop();
      for(int i = 0; i < 4; i++){
        *(pwm_ary + i) = 0;
      }
      sflg = true;
      //Serial.println("pwm:0");
    }else if((int)rcv >= '0' & (int)rcv <= '9'){
      pwm  = convert_pwm((int)rcv - 48);
      for(int i = 0; i < 4; i++){
        *(pwm_ary + i) = pwm;
      }
      out_pwm(pwm_ary);
      if(pwm) sflg = false;
      else sflg = true;
      //Serial.println("pwm:");
      //Serial.println(pwm);
    }
  }
  if(flag){
    digitalWrite(53, HIGH);

    Serial.write("x:");Serial.println(xyt[0],5);
    Serial.write("y:");Serial.println(xyt[1],5);
    Serial.write("T:");Serial.println(xyt[2],5);
    
    //1[m] 進んだかの確認
//    if(xyt[0] > 1.00){
//      Serial.println("1.00 susunndayo");
//      motorstop();
//      for(int i = 0; i < 4; i++){
//        *(pwm_ary + i) = 0;
//      }
//    }
    Wire.beginTransmission(ADDR);
    Wire.write(ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR,14,true);  // request a total of 14 registers
    AcX=(int16_t)(Wire.read()<<8|Wire.read());  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=(int16_t)Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=(int16_t)Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=(int16_t)Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=(int16_t)Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=(int16_t)Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=(int16_t)Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    //Serial.print("origAcX = ");Serial.print(AcX);
    AcX -= avAcX;
    AcX = (AcX > 100 || AcX < -100) ? AcX : 0; 
  //  AcY = (AcY > 1000 || AcY < -1000) ? AcY : 0; 
  //  GyZ = (GyZ > 1000 || GyZ < -1000) ? GyZ : 0; 

    Acc = AcX * 9.80665 / 8192.0; //LSB sensitivity 16384 LSB/g センサの生データ→重力加速度[g]→加速度[m/s2]
    lpv = lpv * fc + Acc * (1 - fc); //ローパスフィルタ　重力加速度の抽出
    hpv = Acc - lpv; //ハイパスフィルタ （加速度＋重力加速度) - 重力加速度
  
    //speed
    Sp = ((hpv + oldAcc) * slptime) / 2 + Sp;
    oldAcc = hpv;
    if(sflg /*&& !AcX*/)Sp = 0.0000;
    //変位
    x = ((Sp + oldSp) * slptime) / 2 + x;
    oldSp = Sp;


    //Serial.print(" | AcX = "); Serial.print(AcX); 
//    Serial.print("AcY:"); Serial.println(AcY); //G 9.8m/ss
//    Serial.print(" | GyY = "); Serial.print(GyY);
//    Serial.print("GyZ:"); Serial.println(GyZ); //deg/s
    Serial.print("Acc:"); Serial.println(Acc,8); //m/s2
    Serial.print("lpv:"); Serial.println(lpv, 5);
    Serial.print("hpv:"); Serial.println(hpv, 5);
    Serial.print("spd:"); Serial.println(Sp, 5);
    Serial.print("acx:"); Serial.println(-x, 5); //sensorのつける向き逆？

    //Serial.print(" | y = "); Serial.print(y, 5);
    //Serial.print(" | deg = "); Serial.print(deg, 5);
    Serial.println("");
    flag = false;
    digitalWrite(53, LOW);
    //xyt[0] = 0; //xyt[1] = 0; xyt[3] = 0;
  }



}


/*
encoder phase A,B
AB:**
forward :00 01 11 10 00 ...
backward:00 10 11 01 00 ...

*/

const char table[4][4] = {
  {0, -1, 1, 0},
  {1, 0, 0, -1},
  {-1, 0, 0, 1},
  {0, 1, -1, 0}
};

ISR(PCINT2_vect){
  buf = PINK;
  count[0] += table[ old & 0b00000011      ][ buf & 0b00000011      ];
  count[1] += table[(old & 0b00001100) >> 2][(buf & 0b00001100) >> 2];
  count[2] += table[(old & 0b00110000) >> 4][(buf & 0b00110000) >> 4];
  count[3] += table[(old & 0b11000000) >> 6][(buf & 0b11000000) >> 6];
  old = PINK;
  //flag = true;
  /*
   * count0:front_left
   * count1:back_left
   * count2:front_right
   * count3:back_right
   */
  
}

/*
 * 各車輪の角速度（速度）からロボットの速度を求める運動学
 * lx:中心から車輪の軸までのｘ軸の距離
 * ly:中心から車輪の軸までのｙ軸の距離
 * |x|      |    1         1         -1        -1   ||v0|
 * |y| = 1/4|   -1         1         -1         1   ||v1|
 * |θ|      |1/(lx+ly) 1/(lx+ly) 1/(lx+ly) 1/(lx+ly)||v2|
 *                                                   |v3|
 *                           A_mat                   
 *count[]:
 *  0:v1
 *  1:v2
 *  2:v3
 *  3:v0 ??
*/

const double A_mat[3][4] = {
  {0.25, 0.25, -0.25, -0.25},
  {-0.25, 0.25, -0.25, 0.25},
  {-1/(4*(lx+ly)), -1/(4*(lx+ly)), -1/(4*(lx+ly)), -1/(4*(lx+ly))}
};
// ３行目　全部にマイナス→なんか回転方向違った


unsigned char t_count = 0;

ISR(TIMER2_COMPA_vect){
  
  if(t_count == 0){ // 7割り込みごとに１回 割り込み周期0.014272*7=0.099904[s] だいたい0.1[s]
    flag = true;
    
    xyt[0] += (A_mat[0][0]*wheel_k*count[0] + //0.00408:  2*3.14159/1540 = 0.004079987... (これにcountかければラジアン求まる) 
               A_mat[0][1]*wheel_k*count[1] +
               A_mat[0][2]*wheel_k*count[2] +
               A_mat[0][3]*wheel_k*count[3]) * 0.1;
    xyt[1] += (A_mat[1][0]*wheel_k*count[0] +
               A_mat[1][1]*wheel_k*count[1] +
               A_mat[1][2]*wheel_k*count[2] +
               A_mat[1][3]*wheel_k*count[3]) * 0.1;
    xyt[2] += (A_mat[2][0]*wheel_k*count[0] +
               A_mat[2][1]*wheel_k*count[1] +
               A_mat[2][2]*wheel_k*count[2] +
               A_mat[2][3]*wheel_k*count[3]) * 0.1;
//    Serial.print("x:");Serial.println(xyt[0]);
//    Serial.print("y:");Serial.println(xyt[1]);
//    Serial.print("θ:");Serial.println(xyt[2]);
//    Serial.println(A_mat[0][0]*wheel_k*count[0] +
//                   A_mat[0][1]*wheel_k*count[1] +
//                   A_mat[0][2]*wheel_k*count[2] +
//                   A_mat[0][3]*wheel_k*count[3]); //x
//    Serial.println(A_mat[1][0]*wheel_k*count[0] +
//                   A_mat[1][1]*wheel_k*count[1] +
//                   A_mat[1][2]*wheel_k*count[2] +
//                   A_mat[1][3]*wheel_k*count[3]); //y
//    Serial.println(A_mat[2][0]*wheel_k*count[0] +
//                   A_mat[2][1]*wheel_k*count[1] +
//                   A_mat[2][2]*wheel_k*count[2] +
//                   A_mat[2][3]*wheel_k*count[3]); //θ
    count[0] = 0;
    count[1] = 0;
    count[2] = 0;
    count[3] = 0;
    t_count = 0;
  }else {
    t_count ++;
  }
}

int convert_pwm(unsigned char pwm){
  return (255 * pwm) / 9;
}

void out_pwm(unsigned char *pwmary){
  for(int i = 0; i < 4; i ++){
    analogWrite(*(pwm_pin + i), *(pwmary + i));
  }
}

void north(){
  digitalWrite(AF, HIGH);
  digitalWrite(AB, LOW);
  digitalWrite(BF, HIGH);
  digitalWrite(BB, LOW);
  digitalWrite(CF, HIGH);
  digitalWrite(CB, LOW);
  digitalWrite(DF, HIGH);
  digitalWrite(DB, LOW);
}

void west(){
  digitalWrite(AF, HIGH);
  digitalWrite(AB, LOW);
  digitalWrite(BF, LOW);
  digitalWrite(BB, HIGH);
  digitalWrite(CF, HIGH);
  digitalWrite(CB, LOW);
  digitalWrite(DF, LOW);
  digitalWrite(DB, HIGH);
  
}

void south(){
  digitalWrite(AF, LOW);
  digitalWrite(AB, HIGH);
  
  digitalWrite(BF, LOW);
  digitalWrite(BB, HIGH);
  
  digitalWrite(CF, LOW);
  digitalWrite(CB, HIGH);
  
  digitalWrite(DF, LOW);
  digitalWrite(DB, HIGH);
  
}

void east(){
  digitalWrite(AF, LOW);
  digitalWrite(AB, HIGH);
  
  digitalWrite(BF, HIGH);
  digitalWrite(BB, LOW);
  
  digitalWrite(CF, LOW);
  digitalWrite(CB, HIGH);
  
  digitalWrite(DF, HIGH);
  digitalWrite(DB, LOW);
  
}

void northwest(){
  digitalWrite(AF, HIGH);
  digitalWrite(AB, LOW);
  
  digitalWrite(BF, LOW);
  digitalWrite(BB, LOW);
  
  digitalWrite(CF, HIGH);
  digitalWrite(CB, LOW);
  
  digitalWrite(DF, LOW);
  digitalWrite(DB, LOW);
}

void northeast(){
  digitalWrite(AF, LOW);
  digitalWrite(AB, LOW);
  
  digitalWrite(BF, HIGH);
  digitalWrite(BB, LOW);
  
  digitalWrite(CF, LOW);
  digitalWrite(CB, LOW);
  
  digitalWrite(DF, HIGH);
  digitalWrite(DB, LOW);
}

void southeast(){
  digitalWrite(AF, LOW);
  digitalWrite(AB, HIGH);
  
  digitalWrite(BF, LOW);
  digitalWrite(BB, LOW);
  
  digitalWrite(CF, LOW);
  digitalWrite(CB, HIGH);
  
  digitalWrite(DF, LOW);
  digitalWrite(DB, LOW);
}

void southwest(){
  digitalWrite(AF, LOW);
  digitalWrite(AB, LOW);
  
  digitalWrite(BF, LOW);
  digitalWrite(BB, HIGH);
  
  digitalWrite(CF, LOW);
  digitalWrite(CB, LOW);
  
  digitalWrite(DF, LOW);
  digitalWrite(DB, HIGH);
}

void leftrotate(){
  digitalWrite(AF, HIGH);
  digitalWrite(AB, LOW);
  
  digitalWrite(BF, HIGH);
  digitalWrite(BB, LOW);
  
  digitalWrite(CF, LOW);
  digitalWrite(CB, HIGH);
  
  digitalWrite(DF, LOW);
  digitalWrite(DB, HIGH);
}

void rightrotate(){
  digitalWrite(AF, LOW);
  digitalWrite(AB, HIGH);
  
  digitalWrite(BF, LOW);
  digitalWrite(BB, HIGH);
  
  digitalWrite(CF, HIGH);
  digitalWrite(CB, LOW);
  
  digitalWrite(DF, HIGH);
  digitalWrite(DB, LOW);
}

void motorstop(){
  digitalWrite(AF, LOW);
  digitalWrite(AB, LOW);
  digitalWrite(BF, LOW);
  digitalWrite(BB, LOW);
  digitalWrite(CF, LOW);
  digitalWrite(CB, LOW);
  digitalWrite(DF, LOW);
  digitalWrite(DB, LOW);
  for(int i = 0; i < 4; i++){
    analogWrite(*(pwm_pin + i), 0);
  }
}
