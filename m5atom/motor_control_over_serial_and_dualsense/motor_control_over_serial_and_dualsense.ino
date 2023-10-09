#include <ps5Controller.h> //https://github.com/rodneybakiskan/ps5-esp32

#include <DDT_Motor_M15M06.h> //https://github.com/takex5g/M5_DDTMotor_M15M06
#include <M5Atom.h>

#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

#define EMERGENCY_MONITOR (GPIO_NUM_25) //GPIO25ピンを緊急停止ボタンの電圧モニタに利用
#define EMERGENCY_STOP (0) //ストップ状態
#define NUM_STRIPS 1
#define NUM_LEDS_PER_STRIP 18
#define BRIGHTNESS  255
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];
#define PIN_TO_LEDS 23  // connect to GPIO23

#define REMOTE_CTRL 2
#define AUTONOMOUS 1
#define EMERGENCY 0

// Odometry Difinition
#define BASE_WIDTH  (0.52) //(m)トレッド幅
#define WHEEL_SIZE  (0.067) //(m)ホイールの直径
#define WHEEL_TIRE_SIZE  (0.150) //(m)タイヤをつけた場合の直径１TODO
#define MOTOR_POLES  (15.0) //極数
#define MOTOR_STEPS  (4.0) //4°
struct ODOMETRY{
  bool is_reset_req;
  double last_distance;
  double pos_x;
  double pos_y;
  unsigned long update_millis;
};

int operation_mode = AUTONOMOUS;

// FastLEDライブラリの設定（CRGB構造体）
CRGB dispColor(uint8_t r, uint8_t g, uint8_t b) {
  return (CRGB)((r << 16) | (g << 8) | b);
}
// 色指定配列（1次要素を色番号として表示する色を2次要素の3原色{赤、緑、青}で設定）
//       LED色： {{0:消灯}  , {1:ピンク}   , {2:オレンジ}  , {3:グリーン} , {4:パープル}  , {5:ホワイト}}
int color[][3] = {{0, 0, 0}, {255, 0, 70}, {255, 70, 0}, {70, 255, 0}, {70, 0, 255}, {255, 255, 255}};

// 変数宣言
int num = 0;  //matrix配列の色番号格納用


// LEDマトリクス表示指定配列（0:消灯、1以上：色指定配列で指定した色に指定（color配列1次要素番号））
int matrix[5][5] = {{3,0,0,0,0},
                    {3,3,3,0,0},
                    {3,3,3,3,3},
                    {3,3,3,0,0},
                    {3,0,0,0,0}};

int16_t Speed[2];   // Speed of motor {Right,Left}
int16_t Act_Speed[2];   // Actual Speed of motor {Right,Left}
int16_t Act_Pos[2];   // Actual Position of motor {Right,Left}
int16_t Act_Pos_Offset[2];   // Actual Position of motor {Right,Left}
uint8_t Acce = 6;    // Acceleration of motor 実験の結果6ぐらいが最適
uint8_t Brake_Disable = 0; // Brake position of motor
uint8_t Brake_Enable = 0xFF;
uint8_t ID = 1;      // ID of Motor (default:1)

Receiver Rcv;
// M5Stackのモジュールによって対応するRX,TXのピン番号が違うためM5製品とRS485モジュールに対応させてください
auto motor_handler = MotorHandler(32, 26); // RX,TX (StickC Plus:33, 32, ATOM:32, 26)
const int16_t SPEED_MAX = 100; //DDT_M6の最高回転数は200±10rpm
const int16_t AUTO_MAX = 50; //自律走行の最高速度は100rpmに制限　AUTO_MAX <= SPEED_MAX
const int16_t POS_MAX = 32767; //車輪のエンコーダーの最大値
uint8_t brake = Brake_Disable;

ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
ros::Publisher odomPublisher("atom/odometry", &odom_msg);
struct ODOMETRY odom;


//Ref: https://kougaku-navi.hatenablog.com/entry/2021/10/04/155038
// String型のカンマ区切り文字列をint型配列に分解する関数
void stringToIntValues(String str, int value[], char delim) {
  int k = 0;
  int j = 0;
  char text[8];

  for (int i = 0; i <= str.length(); i++) {
    char c = str.charAt(i);
    if ( c == delim || i == str.length() ) {
      text[k] = '\0';
      value[j] = atoi(text);
      j++;
      k = 0;
    } else {
      text[k] = c;
      k++;
    }
  }
}

void connect_dualsense(){
  ps5.begin("4C:B9:9B:64:76:1A"); //replace with your MAC address
  esp_log_level_set("ps5_L2CAP", ESP_LOG_VERBOSE);
  esp_log_level_set("ps5_SPP", ESP_LOG_VERBOSE);  
  //Serial.println("Ready.");
  ps5.setLed(0, 255, 0); //Set LED to Green
  ps5.setFlashRate(1000,1000); // Range: 0->2550 ms, Set to 0, 0 for the light to remain on
  ps5.sendToController(); //Send LED color to DualSense
}

void setup()
{
  M5.begin(false, false, true); //本体初期化（UART, I2C, LED）
  pinMode(EMERGENCY_MONITOR, INPUT); 
  gpio_pulldown_dis(EMERGENCY_MONITOR);

  for (int i = 0; i < 5; i++) {     //matrix行ループ
    for (int j = 0; j < 5; j++) {   //matrix列ループ
      num = matrix[i][j];           //matrix配列の色番号取得
      //matrix行ごと（0,5,10,15,20）に左から右へ色番号の色で表示
      M5.dis.drawpix(i*5+j, dispColor(color[num][0], color[num][1], color[num][2]));
    }
  }

  FastLED.addLeds<NEOPIXEL, PIN_TO_LEDS>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.setBrightness( BRIGHTNESS );
  fill_solid(leds[0],NUM_LEDS_PER_STRIP, CRGB( 255, 255, 255));
  FastLED.show();

  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  //Serial.println("DDT-Motor RS485");
  //ID = 1; //IDをセット　ID 1=Wheel R, ID 2=Wheel L
  // for (int i=0;i<5;i++){  //5回コマンドを送るとIDがモーターに保存され次回以降は設定不要になる。2台以上を同じバスにつないでいる場合は初回に設定必要。
  //   motor_handler.Set_MotorID(ID);
  // }
  operation_mode = AUTONOMOUS;
  motor_stop();

  connect_dualsense();

  ros_setup();
}

bool toggle_flag = true;
bool init_odometry_flag = true;
void loop()
{
  int emergency_status = digitalRead(EMERGENCY_MONITOR);

  if (emergency_status == EMERGENCY_STOP){
    motor_brake();
    //Serial.println("Emergency Stop");
  }
  else{    
    if (ps5.isConnected() == true) {
      if (ps5.Options()){
        operation_mode = AUTONOMOUS;
        // ps5.setLed(255, 0, 0); //Set LED to Red
        // //Serial.println("Command Mode");
        // ps5.setFlashRate(1000,1000); // Range: 0->2550 ms, Set to 0, 0 for the light to remain on
        // ps5.sendToController(); //Send LED color to DualSense」
        delay(5);
      }
      else if (ps5.Share()){
        operation_mode = REMOTE_CTRL;
        // ps5.setLed(0, 255, 0); //Set LED to Green
        // //Serial.println("Remote Control Mode");
        // ps5.setFlashRate(1000,1000); // Range: 0->2550 ms, Set to 0, 0 for the light to remain on
        // ps5.sendToController(); //Send LED color to DualSense
        delay(5);
      }

      //set Motor Speed on each Operation Mode
      if (ps5.Cross()){
        //Serial.println("Brake");
        motor_brake();
      } 
      else if(operation_mode == REMOTE_CTRL){
        operation_mode = REMOTE_CTRL;
        if (ps5.R1() && ps5.L1()) { //左右スティックモード
          int left_vel = (int)(SPEED_MAX*ps5.LStickY()/128);
          int right_vel = (int)(SPEED_MAX*ps5.RStickY()/128);
          vehicle_run(right_vel,left_vel);
          // Serial.print("Run ");
          // Serial.print(left_vel);
          // Serial.print(",");
          // Serial.print(right_vel);
          // Serial.println();
        }
        else if(ps5.L2()){ //左スティックモード
          int speed_limit = SPEED_MAX; //通常の走行では100で十分
          int rotation_limit = (int)(SPEED_MAX*0.3);
          if(ps5.R2()){
            speed_limit = SPEED_MAX;rotation_limit = (int)(SPEED_MAX*0.4);
          }
          int val_LStickX = ps5.LStickX();
          int val_LStickY = ps5.LStickY();
          //if(val_LStickY <= -20)val_LStickX = -val_LStickX; //バックする時はスティックの値を反転
          if(val_LStickX > -16 && val_LStickX < 16)val_LStickX = 0;
          if(val_LStickY > -20 && val_LStickY < 20)val_LStickY = 0;

          int left_vel = (int)(speed_limit*val_LStickY/128)+(int)(rotation_limit*val_LStickX/128);
          int right_vel = (int)(speed_limit*val_LStickY/128)-(int)(rotation_limit*val_LStickX/128);
          vehicle_run(right_vel,left_vel);
          // Serial.print("Run ");
          // Serial.print(left_vel);
          // Serial.print(",");
          // Serial.print(right_vel);
          // Serial.println();
        } 
        else {
        //  Serial.println("Free");
         motor_stop();
        }
      }

      /*
      if (ps5.Right()) Serial.println("Right Button");
      if (ps5.Down()) Serial.println("Down Button");
      if (ps5.Up()) Serial.println("Up Button");
      if (ps5.Left()) Serial.println("Left Button");
      if (ps5.Square()) Serial.println("Square Button");
      if (ps5.Cross()) Serial.println("Cross Button");
      if (ps5.Circle()) Serial.println("Circle Button");
      if (ps5.Triangle()) Serial.println("Triangle Button");
      if (ps5.UpRight()) Serial.println("Up Right");
      if (ps5.DownRight()) Serial.println("Down Right");
      if (ps5.UpLeft()) Serial.println("Up Left");
      if (ps5.DownLeft()) Serial.println("Down Left");
      if (ps5.L1()) Serial.println("L1 Button");
      if (ps5.R1()) Serial.println("R1 Button");
      if (ps5.Share()) Serial.println("Share Button");
      if (ps5.Options()) Serial.println("Options Button");
      if (ps5.L3()) Serial.println("L3 Button");
      if (ps5.R3()) Serial.println("R3 Button");
      if (ps5.PSButton()) Serial.println("PS Button");
      if (ps5.Touchpad()) Serial.println("Touch Pad Button");
      if (ps5.L2()) {
        Serial.printf("L2 button at %d\n", ps5.L2Value());
      }
      if (ps5.R2()) {
        Serial.printf("R2 button at %d\n", ps5.R2Value());
      }
      if (ps5.LStickX()) {
        Serial.printf("Left Stick x at %d\n", ps5.LStickX());
      }
      if (ps5.LStickY()) {
        Serial.printf("Left Stick y at %d\n", ps5.LStickY());
      }
      if (ps5.RStickX()) {
        Serial.printf("Right Stick x at %d\n", ps5.RStickX());
      }
      if (ps5.RStickY()) {
        Serial.printf("Right Stick y at %d\n", ps5.RStickY());
      }
      // This delay is to make the output more human readable
      // Remove it when you're not trying to see the output
      //delay(300);*/
    }
  }

  if(operation_mode == AUTONOMOUS){
    if (Serial.available() > 0) {
      String text = Serial.readStringUntil('\n');
      //Ex:100,-200: Right Motor = Speed 100 / Left Motor = Speed -200
      //Serial.println(text);
      int data[3];
      stringToIntValues( text, data, ',' );

      int cmd_speed[2]={0,0};
      switch(data[0]) {
        case 1: //Move
          for(int i = 0; i<=1; i++){
            delay(5);
            int sp = data[i+1];
            //入力データ範囲Check＆速度制限
            if(sp <= AUTO_MAX && sp >= -AUTO_MAX)cmd_speed[i]=sp;
            else if(sp>AUTO_MAX)cmd_speed[i]=AUTO_MAX;
            else if(sp<-AUTO_MAX)cmd_speed[i]=-AUTO_MAX;
            else cmd_speed[i]=0;
          }
          vehicle_run(cmd_speed[0],cmd_speed[1]);
          break;
        case 2: //Stop
          motor_stop();
          break;
        case 3: //Stop
          motor_brake();
          break;
        case 4: //Initialize Odometory
          init_odometry_flag = true;
          break;
        case 5: //Get Odometory
          break;
        case 6: //Get Current Velocity
          break;
        case 7: //Get Angle
          break;
      }
    }
  }

  //Set motor speed and report
  int wheel_sp_L = 0; 
  int wheel_sp_R = 0;
  int wheel_pos_L = 0;
  int wheel_pos_R = 0;

  ID=1; //Wheel R
  // Serial.print("Speed[1]:");
  // Serial.print(Speed[0]);
  motor_handler.Control_Motor(Speed[0], ID, Acce, brake, &Rcv);//スピード0：モーター停止
  delay(5);//1回の通信ごとに5msのWaitが必要（RS485の半二重通信の問題と思われる）
  Serial.print("{\"R_VEL\":");
  wheel_sp_R=Rcv.BSpeed;
  Act_Speed[0]=wheel_sp_R;
  Serial.print(wheel_sp_R);
  Serial.print(",\"R_POS\":");
  wheel_pos_R=POS_MAX-Rcv.Position; //モーターの向きに合わせて符号を設定
  Act_Pos[0]=wheel_pos_R;
  Serial.print(wheel_pos_R);
  Serial.print(",");
  ID=2; //Wheel L
  // Serial.print(",Speed[2]:");
  // Serial.println(Speed[1]);
  motor_handler.Control_Motor(Speed[1], ID, Acce, brake, &Rcv);//スピード0：モーター停止
  Serial.print("\"L_VEL\":");
  wheel_sp_L=-Rcv.BSpeed; //モーターの向きに合わせて符号を設定
  Act_Speed[1]=wheel_sp_L;
  Serial.print(wheel_sp_L);
  Serial.print(",\"L_POS\":");
  wheel_pos_L=Rcv.Position; 
  Act_Pos[1]=wheel_pos_L;
  Serial.print(wheel_pos_L);
  Serial.println("}");
  delay(5);

  ros_update(wheel_sp_L, wheel_sp_R, wheel_pos_L, wheel_pos_R);


  //TODO: オドメトリを計算する（エンコーダー値0-32767を角度に変換、距離・Yaw角度に変換（タイヤのサイズ、車幅が必要）、原点リセットコマンド（現在のオドメトリを保持）を作る）
  //TODO: ROS2との接続
  //TODO: Spurコマンド的な何か（ROSのナビゲーションの基本を調査する）
  //TODO: ナビゲーション (障害物で停止、指定の距離進む)
  //

  //Indicator LED
  EVERY_N_MILLISECONDS(500) {
    for(int x = 0; x < NUM_STRIPS; x++) {
      // This inner loop will go over each led in the current strip, one at a time
        if(toggle_flag){
          CRGB color = CRGB::Red;
          if(operation_mode == AUTONOMOUS)color = CRGB::Blue;
          else if(operation_mode == REMOTE_CTRL)color = CRGB::Yellow;
          else color = CRGB::Red;
          fill_solid(leds[x], NUM_LEDS_PER_STRIP, color);
          toggle_flag=false;
        }
        else {
          fill_solid(leds[x], NUM_LEDS_PER_STRIP, CRGB::Black);
          toggle_flag=true;
        }
        FastLED.show();
    }
  }

}

void motor_stop(){
  Speed[0]=0;
  Speed[1]=0;
  brake = Brake_Disable;
}

void motor_brake(){
  Speed[0]=0;
  Speed[1]=0;
  brake = Brake_Enable;
}

void vehicle_run(int right, int left){
  Speed[0]=-right;
  Speed[1]=left;
  brake = Brake_Disable;
} 

void ros_setup(){
  nh.initNode();
  nh.advertise(odomPublisher);
  odometry_reset(&odom);
}

void ros_update(int speed_left, int speed_right, int position_left, int position_right){
  odometry_udate(speed_left, speed_right); // write odom_msg
  odomPublisher.publish(&odom_msg);
  nh.spinOnce();
}

void odometry_reset(struct ODOMETRY* odom){
  odom->is_reset_req = false;
  odom->last_distance = 0.0;
  odom->pos_x= 0.0;
  odom->pos_y = 0.0;
  odom->update_millis = millis();
}

void odometry_udate(int speed_left, int speed_right){
  if(odom.is_reset_req){
    odometry_reset(&odom);
    return;
  }
  unsigned long interval_millis, now_millis;
  now_millis = millis();
  if(now_millis >= odom.update_millis){
    interval_millis = now_millis - odom.update_millis;
  }else{
    interval_millis = ~(odom.update_millis - now_millis) + 1;
  }

  double left_velocity, right_velocity, left_distance, right_distance, distance, theta, d_x, d_y;
  geometry_msgs::Quaternion quat;

  left_velocity = (double)speed_left * WHEEL_TIRE_SIZE / WHEEL_SIZE;
  right_velocity = (double)speed_right * WHEEL_TIRE_SIZE / WHEEL_SIZE;
  left_distance = left_velocity * interval_millis / 1000;
  right_distance = right_velocity * interval_millis / 1000;
  distance = (left_distance + right_distance / 2.0);
  theta = (right_distance - left_distance) / BASE_WIDTH;;
  d_x = (distance - odom.last_distance) * cos(theta);
  d_y = (distance - odom.last_distance) * sin(theta);
  quat = tf::createQuaternionFromYaw(theta);

  odom.pos_x += d_x;
  odom.pos_y += d_y;  
  odom.last_distance = distance;
  odom.update_millis = now_millis;
  
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "/odometry";
  odom_msg.child_frame_id = "base_footprint";

  /*位置情報を入れる*/
  odom_msg.pose.pose.position.x = odom.pos_x;
  odom_msg.pose.pose.position.y = odom.pos_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;

  odom_msg.twist.twist.linear.x = (left_velocity + right_velocity) / 2.0;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.z = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = (right_velocity - left_velocity) / BASE_WIDTH;
}

