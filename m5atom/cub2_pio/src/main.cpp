#include <ps5Controller.h> //https://github.com/rodneybakiskan/ps5-esp32
#include <DDT_Motor_M15M06.h> //https://github.com/takex5g/M5_DDTMotor_M15M06
#include <M5Unified.h>
#include <FastLED.h>   // FastLED(RGB LED)のライブラリを使用可能にします。

//#define EMG_EN_PIN 39 //Emergency Enable for PS board

int16_t Speed[4];   // Speed of motor {Right,Left}
uint8_t Brake_Disable = 0; // Brake position of motor
uint8_t Brake_Enable = 0xFF;
uint8_t Acce = 2;    // Acceleration of motor
uint8_t Brake_P = 0; // Brake position of motor
uint8_t ID = 1;      // ID of Motor (default:1)
uint8_t brake = Brake_Disable;
const int16_t POS_MAX = 32767; //車輪のエンコーダーの最大値
Receiver Receiv;
// M5Stackのモジュールによって対応するRX,TXのピン番号が違うためM5製品とRS485モジュールに対応させてください
auto motor_handler = MotorHandler(33, 23); // Cub2 ATOM(33, 23) Cub1 RX,TX ATOM(32, 26) DDSM210 ATOM S3(2,1)

// RGB LEDの数を指定(M5Atom Matrixなら25)
#define NUM_LEDS 25
CRGB leds[NUM_LEDS];
#define LED_DATA_PIN 27


void connect_dualsense(){
  ps5.begin("4C:B9:9B:64:76:1A"); //replace with your MAC address
  esp_log_level_set("ps5_L2CAP", ESP_LOG_VERBOSE);
  esp_log_level_set("ps5_SPP", ESP_LOG_VERBOSE);  
  //Serial.println("Ready.");
  ps5.setLed(0, 255, 0); //Set LED to Green
  ps5.setFlashRate(1000,1000); // Range: 0->2550 ms, Set to 0, 0 for the light to remain on
  ps5.sendToController(); //Send LED color to DualSense
}
void motor_exec(){
  for(int i=0;i<4;i++){
    motor_handler.Control_Motor(Speed[i], i+1, Acce, brake, &Receiv);//スピード0：モーター停止
    delay(5);//1回の通信ごとに5msのWaitが必要（RS485の半二重通信の問題と思われる）
  }
}

void motor_stop(){
  Speed[0]=Speed[2]=0;
  Speed[1]=Speed[3]=0;
  brake = Brake_Disable;
  motor_exec();
}

void motor_brake(){
  Speed[0]=Speed[2]=0;
  Speed[1]=Speed[3]=0;
  brake = Brake_Enable;
  motor_exec();
}

void vehicle_run(int right, int left){
  Serial.print("lr value ");
  Serial.print(right);
  Serial.print(",");
  Serial.println(left);
  Speed[0]=Speed[2]=-right;
  Speed[1]=Speed[3]=left;
  brake = Brake_Disable;
  motor_exec();
} 


void vehicle_omni(int rf_vel, int rb_vel, int lf_vel, int lb_vel){
  Speed[0]=-rf_vel;
  Speed[1]=lf_vel;
  Speed[2]=-rb_vel;
  Speed[3]=lb_vel;
  brake = Brake_Disable;
  motor_exec();
} 

void setup()
{
  // auto cfg = M5.config();       // M5Stack初期設定用の構造体を代入
  // M5.begin(cfg);                                           // M5デバイスの初期化 //このままだとDDTモーターと処理が衝突する
  FastLED.addLeds<WS2811, LED_DATA_PIN, GRB>(leds, NUM_LEDS);   // RGB LEDを初期設定
  FastLED.setBrightness(20);                               // 明るさを設定（20以上は熱で壊れる可能性あり。）
  leds[0] = CRGB::Blue;                      // LED[0]をGreenに設定
  FastLED.show();                           // LEDを表示

  Serial.begin(115200);
  Serial.println("DDT-Motor RS485");
//  pinMode(EMG_EN_PIN, OUTPUT);
//  digitalWrite(EMG_EN_PIN, HIGH);

  for(int i=1;i<=4;i++){
    motor_handler.Control_Motor(0, i, Acce, Brake_P, &Receiv); //4つのモーター
  }
  connect_dualsense();
  leds[0] = CRGB::Green;                      // LED[0]をGreenに設定
  FastLED.show();                           // LEDを表示
}

int last_num_sign[2] = {0,0};
const int16_t SPEED_MAX = 115;  //DDSM115 115rpm = max11
const int16_t SPEED_MIN = -115;
void loop()
{
  leds[0] = CRGB::Green;                      // LED[0]をGreenに設定

  if (ps5.isConnected() == true) {  
    leds[0] = CRGB::Red;                      // LED[0]をGreenに設定
    if (ps5.Cross()){ //バツボタンでブレーキ 遠隔操作モードではすぐ反応
      motor_brake();
    }  
    else if(ps5.L2()){ //左スティックモード
      int speed_limit = SPEED_MAX; //通常の走行では100で十分
      int rotation_limit = (int)(SPEED_MAX*0.3);
      int val_LStickX = ps5.LStickX();
      int val_LStickY = ps5.LStickY();
      if(val_LStickX > -16 && val_LStickX < 16)val_LStickX = 0;
      if(val_LStickY > -20 && val_LStickY < 20)val_LStickY = 0;

      int left_vel = (int)(speed_limit*val_LStickY/128)+(int)(rotation_limit*val_LStickX/128);
      int right_vel = (int)(speed_limit*val_LStickY/128)-(int)(rotation_limit*val_LStickX/128);
      vehicle_run(right_vel,left_vel);
    } 
    else if(ps5.R2()){ //右スティックモード
      int speed_limit = SPEED_MAX; //通常の走行では100で十分
      int val_RStickX = ps5.RStickX();
      int val_RStickY = ps5.RStickY();

      int rf_vel = -(int)(speed_limit*val_RStickX/128)+(int)(speed_limit*val_RStickY/128);
      int rb_vel = (int)(speed_limit*val_RStickX/128)+(int)(speed_limit*val_RStickY/128);
      int lf_vel = (int)(speed_limit*val_RStickX/128)+(int)(speed_limit*val_RStickY/128);
      int lb_vel = -(int)(speed_limit*val_RStickX/128)+(int)(speed_limit*val_RStickY/128);
      vehicle_omni(rf_vel,rb_vel,lf_vel,lb_vel);
    } 
    else {
      motor_stop();
    }
  }
  FastLED.show();                           // LEDを表示
}


