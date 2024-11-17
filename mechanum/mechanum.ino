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

#define wheel_k 0.001632 //車輪の回転数から速度を出す係数k
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
 *   = 0.001143497758*count　[m/s]
 */

const double res = GEAR * PPR * 4; //encoder resolution

//GYT521 (MPU6050)
#define CONFIG 0x1A
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_ZOUT_H 0x47
#define ADDR 0x68
#define PWR_MGMT_1 0x6B

//sensor
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ, avAcX = 0, avAcY = 0, avGyZ = 0;
double x = 0, y = 0, rad = 0;
double slptime = 0.1;

//filter
double fc = 0.6; //filter coefficient ローパスフィルタの係数　1に近づけるほど平滑化の度合いが大きい
double lpvx = 0, lpvy = 0; //low-pass filter value
double hpvx = 0, hpvy = 0; //high-pass filter value
double Accx = 0, Spx = 0, Accy = 0, Spy = 0; //加速度，速度
double oldAccx = 0, oldSpx = 0, oldAccy = 0, oldSpy = 0; //一つ前の加速度，速度
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
  Wire.write(CONFIG);  // PWR_MGMT_1 register DLPF_CFG[2-0]
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
    Wire.requestFrom(ADDR,4,true);
    AcX=(int16_t)Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    avAcX += AcX / 10;
    AcY=(int16_t)Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    avAcY += AcY / 10;
    Wire.beginTransmission(ADDR);
    Wire.write(GYRO_ZOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR,2,true);
    GyZ=(int16_t)Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    avGyZ += GyZ / 10;
    delay(100);
    //Serial.println(avAcX); //debug
  }
  
}

char rcv = 'l';
char rcv_pwm[4] = {0, 0, 0, 0};
unsigned char pwm = 0;
unsigned char pwm_ary[4] = {0, 0, 0, 0}; //A, B, C, D
unsigned char pwm_pin[4] = {APWM, BPWM, CPWM, DPWM};
int count[4] = {0,0,0,0}; //encoder pulse counter
unsigned char old = 0;
unsigned char buf; 
bool flag = false;

double enxyt[3] = {0,0,0};
double acxyt[3] = {0,0,0};
double abxyt[3] = {0,0,0};

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
    }else if(rcv == 'r'){ //all reset
      for(int i = 0; i < 10; i++){
        Wire.beginTransmission(ADDR);
        Wire.write(ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(ADDR,4,true);
        AcX=(int16_t)Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
        avAcX += AcX / 10;
        AcY=(int16_t)Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        avAcY += AcY / 10;
        Wire.beginTransmission(ADDR);
        Wire.write(GYRO_ZOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(ADDR,2,true);
        GyZ=(int16_t)Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        avGyZ += GyZ / 10;
        delay(100);
        //Serial.println(avAcX); //debug
      }
      enxyt[0] = 0; enxyt[1] = 0; enxyt[2] = 0;
      x = 0; y = 0; rad = 0;
    }else if(rcv == 'n'){ //navigation
      //nを受け取ったあと、４つのモータのPWM受けとる
      while(Serial.available()!=4);
      if(Serial.available()>=4){ //v0, v1, v2, v3 PWM
        Serial.readBytes(rcv_pwm, 4);
        eachmotor(rcv_pwm);
      }
    }
  }
  if(flag){
    digitalWrite(53, HIGH); //debug
    
    Wire.beginTransmission(ADDR);
    Wire.write(ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR,4,true);  // request a total of 14 registers
    AcX=(int16_t)(Wire.read()<<8|Wire.read());  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=(int16_t)Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    Wire.beginTransmission(ADDR);
    Wire.write(GYRO_ZOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR,2,true);
    GyZ=(int16_t)Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    //Serial.print("origAcX = ");Serial.print(AcX);
    //X軸方向の加速度
    AcX -= avAcX;
    AcX = (AcX > 100 || AcX < -100) ? AcX : 0; 

    Accx = AcX * 9.80665 / 8192.0; //LSB sensitivity 16384 LSB/g センサの生データ→重力加速度[g]→加速度[m/s2]
    lpvx = lpvx * fc + Accx * (1 - fc); //ローパスフィルタ　重力加速度の抽出
    hpvx = Accx - lpvx; //ハイパスフィルタ （加速度＋重力加速度) - 重力加速度
  
    //speed
    Spx = ((Accx + oldAccx) * slptime) / 2 + Spx;
    oldAccx = Accx;
    if(sflg /*&& !AcX*/)Spx = 0.0000;
    //変位
    acxyt[0] = ((Spx + oldSpx) * slptime) / 2 + acxyt[0];
    oldSpx = Spx;

    //Y軸方向の加速度
    //Serial.print("origAcY = ");Serial.print(AcY);
    AcY -= avAcY;
    AcY = (AcY > 100 || AcY < -100) ? AcY : 0; 

    Accy = AcY * 9.80665 / 8192.0; //LSB sensitivity 16384 LSB/g センサの生データ→重力加速度[g]→加速度[m/s2]
    lpvy = lpvy * fc + Accy * (1 - fc); //ローパスフィルタ　重力加速度の抽出
    hpvy = Accy - lpvy; //ハイパスフィルタ （加速度＋重力加速度) - 重力加速度
  
    //speed
    Spy = ((Accy + oldAccy) * slptime) / 2 + Spy;
    oldAccy = Accy;
    if(sflg /*&& !AcY*/)Spy = 0.0000;
    //変位
    acxyt[1] = ((Spy + oldSpy) * slptime) / 2 + acxyt[1];
    oldSpy = Spy;

    /*
    gyro:+- 250 deg/s LSB sensitivity  131 LSB/deg/s 
    deg = pi * rad / 180
    LSB/deg/s = LSB/(pi*rad/180)/s 
    131*180/pi LSB/rad/s
    7505.7471162... LSB/rad/s
    */
    //Serial.print("origGyZ:");Serial.println(GyZ);
    GyZ -= avGyZ;
    GyZ = (GyZ > 10 || GyZ < -10) ?GyZ : 0;
    //Serial.print("modGyZ:");Serial.println(GyZ);
    acxyt[2] = GyZ * slptime / 7506;

    //Serial.write("x:");Serial.println(enxyt[0],5);
    //Serial.write("y:");Serial.println(enxyt[1],5);
    //Serial.write("T:");Serial.println(enxyt[2],5);
    //Serial.print(" | AcX = "); Serial.print(AcX); 
//    Serial.print("AcY:"); Serial.println(AcY); //G 9.8m/ss
//    Serial.print(" | GyY = "); Serial.print(GyY);
//    Serial.print("GyZ:"); Serial.println(GyZ); //deg/s
//    Serial.print("Acc:"); Serial.println(Accx,8); //m/s2
//    Serial.print("lpv:"); Serial.println(lpvx, 5);
//    Serial.print("hpv:"); Serial.println(hpvx, 5);
//    Serial.print("spd:"); Serial.println(Spx, 5);
//    Serial.print("acx:"); Serial.println(-x, 5); //sensorのつける向き逆？
//    Serial.print("acy:"); Serial.println(y, 5);
    x = enxyt[0]*0.856 - acxyt[0] * 0.144; //実測値の誤差　encoder:3.6% acc:21.4%
    y = enxyt[1]*0.856 + acxyt[1] * 0.144; //実測値の誤差　encoder:3.6% acc:21.4%
    rad = (enxyt[2] + acxyt[2]) / 2; 
    Serial.print("x:");Serial.println(x, 5); 
    Serial.print("y:");Serial.println(y, 5); 
    Serial.print("T:");Serial.println(rad, 5); 
//    abxyt[0] = x*cos(rad) + y*cos(rad + 1.5707963268) //y軸はx軸と９０度 PI / 2 = 1.570796326794...
//    abxyt[1] = x*sin(rad) + y*sin(rad + 1.5707963268);
//    abxyt[2] = rad;
//    Serial.print("abx:");Serial.println(abxyt[0], 5);
//    Serial.print("aby:");Serial.println(abxyt[1], 5);
//    Serial.print("abT:");Serial.println(abxyt[2], 5);
    
    //Serial.print(" | y = "); Serial.print(y, 5);
    //Serial.print(" | deg = "); Serial.print(deg, 5);
    flag = false;
    digitalWrite(53, LOW);
    enxyt[0] = 0; enxyt[1] = 0; enxyt[2] = 0; acxyt[0] = 0; acxyt[1] = 0; acxyt[2] = 0;;
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
  
  if(t_count == 6){ // 7割り込みごとに１回 割り込み周期0.014272*7=0.099904[s] だいたい0.1[s]
    flag = true;
    
    enxyt[0] += (A_mat[0][0]*wheel_k*count[0] + //0.00408:  2*3.14159/1540 = 0.004079987... (これにcountかければラジアン求まる) 
               A_mat[0][1]*wheel_k*count[1] +
               A_mat[0][2]*wheel_k*count[2] +
               A_mat[0][3]*wheel_k*count[3]) * 0.1;
    enxyt[1] += (A_mat[1][0]*wheel_k*count[0] +
               A_mat[1][1]*wheel_k*count[1] +
               A_mat[1][2]*wheel_k*count[2] +
               A_mat[1][3]*wheel_k*count[3]) * 0.1;
    enxyt[2] += (A_mat[2][0]*wheel_k*count[0] +
               A_mat[2][1]*wheel_k*count[1] +
               A_mat[2][2]*wheel_k*count[2] +
               A_mat[2][3]*wheel_k*count[3]) * 0.1;
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

void eachmotor(char* pwm){
//    Serial.print("pwm[0]:");Serial.println(pwm[0]);
//    Serial.print("pwm[1]:");Serial.println(pwm[1]);
//    Serial.print("pwm[2]:");Serial.println(pwm[2]);
//    Serial.print("pwm[3]:");Serial.println(pwm[3]);
    if(pwm[0] > 0){
      digitalWrite(AF, HIGH);
      digitalWrite(AB, LOW);
      analogWrite(pwm_pin[0], (uint8_t)pwm[0] * 2);
    }else{
      digitalWrite(AF, LOW);
      digitalWrite(AB, HIGH);
      analogWrite(pwm_pin[0], (uint8_t)pwm[0] * (-2));
    }
    if(pwm[1] > 0){
      digitalWrite(BF, HIGH);
      digitalWrite(BB, LOW);
      analogWrite(pwm_pin[1], (uint8_t)pwm[1] * 2);
    }else{
      digitalWrite(BF, LOW);
      digitalWrite(BB, HIGH);
      analogWrite(pwm_pin[1], (uint8_t)pwm[1] * (-2));
    }
    if(pwm[2] > 0){
      digitalWrite(CF, LOW);
      digitalWrite(CB, HIGH);
      analogWrite(pwm_pin[2], (uint8_t)pwm[2] * 2);
    }else{
      digitalWrite(CF, HIGH);
      digitalWrite(CB, LOW);
      analogWrite(pwm_pin[2], (uint8_t)pwm[2] * 2);
    }
    if(pwm[3] > 0){
      digitalWrite(DF, LOW);
      digitalWrite(DB, HIGH);
      analogWrite(pwm_pin[3], (uint8_t)pwm[3] * 2);
    }else{
      digitalWrite(DF, HIGH);
      digitalWrite(DB, LOW);
      analogWrite(pwm_pin[3], (uint8_t)pwm[3] * 2);
    }
}
