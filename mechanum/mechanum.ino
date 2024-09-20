/************************
 * 2024年度 元木大輔
 * 卒検のメカナムロボのプログラム
 * 4つのMD
 * エンコーダ
 * 
 * 
 *************************/
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
 */



const float res = GEAR * PPR * 4; //encoder resolution
void setup() {
  
  // put your setup code here, to run once:
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
}

char rcv = 'l';
unsigned char pwm = 0;
unsigned char pwm_ary[4] = {0, 0, 0, 0}; //A, B, C, D
unsigned char pwm_pin[4] = {APWM, BPWM, CPWM, DPWM};
int count[4] = {0,0,0,0}; //encoder pulse counter
unsigned char old = 0;
unsigned char buf; 
bool flag = false;
void loop() {
  if(Serial.available()>0){
    rcv = Serial.read();
    if(rcv == 'w'){
      Serial.println("前進");
      north();
    }else if(rcv == 'a'){
      Serial.println("左");
      west();
    }else if(rcv == 's'){
      Serial.println("後退");
      south();
    }else if(rcv == 'd'){
      Serial.println("右");
      east();
    }else if(rcv == 'q'){
      Serial.println("左前");
      northwest();
    }else if(rcv == 'e'){
      Serial.println("右前");
      northeast();
    }else if(rcv == 'c'){
      Serial.println("右後ろ");
      southeast();
    }else if(rcv == 'z'){
      Serial.println("左後ろ");
      southwest();
    }else if(rcv == 'f'){
      Serial.println("左回転");
      leftrotate();
    }else if(rcv == 'g'){
      Serial.println("右回転");
      rightrotate();
    }else if(rcv == 'x'){
      Serial.println("停止");
      motorstop();
      for(int i = 0; i < 4; i++){
        *(pwm_ary + i) = 0;
      }
      Serial.println("pwm:0");
    }else if((int)rcv >= 48 & (int)rcv <= 58){ //'0'~'9'
      pwm  = convert_pwm((int)rcv - 48);
      for(int i = 0; i < 4; i++){
        *(pwm_ary + i) = pwm;
      }
      out_pwm(pwm_ary);
      Serial.print("pwm:");
      Serial.println(pwm);
    }
  }
  if(flag){
    Serial.println(count[0]);
    Serial.println(count[1]);
    Serial.println(count[2]);
    Serial.println(count[3]);
    Serial.print("degree0:");Serial.println(((float)count[0] / res) * 360); //360を2πにすればラジアン
    Serial.print("degree1:");Serial.println(((float)count[1] / res) * 360);
    Serial.print("degree2:");Serial.println(((float)count[2] / res) * 360);
    Serial.print("degree3:");Serial.println(((float)count[3] / res) * 360);
    flag = false;
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
  count[2] += table[(old & 0b00110000) >> 4][(buf & 0b00110000) >> 4]; // -=なのはロボのモータの向き
  count[3] += table[(old & 0b11000000) >> 6][(buf & 0b11000000) >> 6];
  old = PINK;
  flag = true;
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
 * |x|      |   -1        -1         1          1   ||v0|
 * |y| = 1/4|   1         -1         -1         1   ||v1|
 * |θ|      |1/(lx+ly) 1/(lx+ly) 1/(lx+ly) 1/(lx+ly)||v2|
 *                                                   |v3|
 *                           A_mat                   
 *count[]:
 *  0:v1
 *  1:v2
 *  2:v3
 *  3:v0 ??
*/

const float A_mat[3][4] = {
  {-0.25, -0.25, 0.25, 0.25},
  {0.25, -0.25, -0.25, 0.25},
  {1/(4*(lx+ly)), 1/(4*(lx+ly)), 1/(4*(lx+ly)), 1/(4*(lx+ly))}
};

float xyt[3] = {0,0,0};

unsigned char t_count = 0;

ISR(TIMER2_COMPA_vect){
  
  if(t_count == 7){ // 7割り込みごとに１回 割り込み周期0.014272*7=0.099904[s] だいたい0.1[s]
    //flag = true;
    
    xyt[0] += (A_mat[0][0]*wheel_k*count[0] +
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
//    count[0] = 0;
//    count[1] = 0;
//    count[2] = 0;
//    count[3] = 0;
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
  digitalWrite(AF, LOW);
  digitalWrite(AB, HIGH);
  
  digitalWrite(BF, LOW);
  digitalWrite(BB, HIGH);
  
  digitalWrite(CF, HIGH);
  digitalWrite(CB, LOW);
  
  digitalWrite(DF, HIGH);
  digitalWrite(DB, LOW);
}

void rightrotate(){
  digitalWrite(AF, HIGH);
  digitalWrite(AB, LOW);
  
  digitalWrite(BF, HIGH);
  digitalWrite(BB, LOW);
  
  digitalWrite(CF, LOW);
  digitalWrite(CB, HIGH);
  
  digitalWrite(DF, LOW);
  digitalWrite(DB, HIGH);
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
