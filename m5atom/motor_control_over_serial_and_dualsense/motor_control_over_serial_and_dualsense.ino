#include <ps5Controller.h> //https://github.com/rodneybakiskan/ps5-esp32

#include <DDT_Motor_M15M06.h> //https://github.com/takex5g/M5_DDTMotor_M15M06
#include <M5Atom.h>

#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>

#define EMERGENCY_MONITOR (GPIO_NUM_25) //GPIO25ピンを緊急停止ボタンの電圧モニタに利用
#define EMERGENCY_STOP (0) //ストップ状態
#define NUM_STRIPS 1
#define NUM_LEDS_PER_STRIP 30
#define BRIGHTNESS  255
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];
#define PIN_TO_LEDS 23  // connect to GPIO23

#define REMOTE_CTRL 2
#define AUTONOMOUS 1
#define EMERGENCY 0

// Odometry Difinition
#define BASE_WIDTH  (0.245) //(m)トレッド幅
#define WHEEL_TIRE_RADIUS  (0.1025) //(m)タイヤをつけた場合の直径１TODO
//#define MOTOR_POLES  (15.0) //極数
//#define MOTOR_STEPS  (4.0) //4°
#define M_PI (3.14159)
#define POSITION_UNKNOWN 0xffff

struct ODOMETRY{
  bool is_reset_req;
  double last_theta;
  double pos_x;
  double pos_y;
  double left_distance;
  double right_distance;
  int last_position_left;
  int last_position_right;
};

#define TWIST_RESET_MILLIS 1000
unsigned long twist_last_millis;

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
ros::Subscriber<geometry_msgs::Twist> twistSubscriber("atom/motor_control", &callBack_motor_control);
ros::Subscriber<std_msgs::Empty> resetSubscriber("atom/odometry_reset", &callBack_odometry_reset);

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

  if (emergency_status == EMERGENCY_STOP){//緊急停止ピン=Highの場合
    motor_brake();
    //Indicator LED
    EVERY_N_MILLISECONDS(500) {
      for(int x = 0; x < NUM_STRIPS; x++) {
        // This inner loop will go over each led in the current strip, one at a time
          if(toggle_flag){
            CRGB color = CRGB::Red;
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
  else{ //緊急停止ではない場合
    if (ps5.isConnected() == true) {
      if (ps5.Options()){ //自立走行モード
        operation_mode = AUTONOMOUS;
        delay(5);
      }
      else if (ps5.Share()){
        operation_mode = REMOTE_CTRL; //遠隔操作モード
        delay(5);
      }

      //set Motor Speed on each Operation Mode
      if (ps5.Cross()){ //バツボタンでブレーキ
        motor_brake();
      } 
      else if(operation_mode == REMOTE_CTRL){
        operation_mode = REMOTE_CTRL;
        if (ps5.R1() && ps5.L1()) { //左右スティックモード
          int left_vel = (int)(SPEED_MAX*ps5.LStickY()/128);
          int right_vel = (int)(SPEED_MAX*ps5.RStickY()/128);
          vehicle_run(right_vel,left_vel);
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
        } 
        else {
         motor_stop();
        }
      }
    }

    //Set motor speed and report
    int wheel_sp_L = 0; 
    int wheel_sp_R = 0;
    int wheel_pos_L = 0;
    int wheel_pos_R = 0;

    ID=1; //Wheel R
    motor_handler.Control_Motor(Speed[0], ID, Acce, brake, &Rcv);//スピード0：モーター停止
    delay(5);//1回の通信ごとに5msのWaitが必要（RS485の半二重通信の問題と思われる）
    wheel_sp_R=Rcv.BSpeed;
    wheel_pos_R= (double)((POS_MAX-Rcv.Position) & 0x7ffc) * 360 / POS_MAX; //モーターの向きに合わせて符号を設定

    ID=2; //Wheel L
    motor_handler.Control_Motor(Speed[1], ID, Acce, brake, &Rcv);//スピード0：モーター停止
    wheel_sp_L=-Rcv.BSpeed; //モーターの向きに合わせて符号を設定
    wheel_pos_L= (double)(Rcv.Position & 0x7ffc) * 360 / POS_MAX; 
    delay(5);

    ros_update(wheel_sp_L, wheel_sp_R, wheel_pos_L, wheel_pos_R);

    //Indicator LED
    EVERY_N_MILLISECONDS(500) {
      for(int x = 0; x < NUM_STRIPS; x++) {
        // This inner loop will go over each led in the current strip, one at a time
          if(toggle_flag){
            CRGB color = CRGB::Red;
            if(operation_mode == AUTONOMOUS)color = CRGB( 0, 255, 0);
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
  nh.subscribe(twistSubscriber);
  twist_last_millis = millis();
  nh.subscribe(resetSubscriber);
  nh.advertise(odomPublisher);
  odometry_reset(&odom);
}

void ros_update(double speed_left, double speed_right, double position_left, double position_right){
  if(operation_mode == AUTONOMOUS){
    twist_reset_check();
  }
  odometry_udate(speed_left, speed_right, position_left, position_right); // write odom_msg
  odomPublisher.publish(&odom_msg);
  nh.spinOnce();
  // test
  // geometry_msgs::Twist twist;
  // twist.linear.x = 0.0;
  // twist.angular.z = 0.261799;
  // callBack_motor_control(twist);
}

void twist_reset_check(){
  unsigned long interval_millis, now_millis;
  now_millis = millis();
  if(now_millis >= twist_last_millis){
    interval_millis = now_millis - twist_last_millis;
  }else{
    interval_millis = ~(twist_last_millis - now_millis) + 1;
  }
  if(twist_last_millis > TWIST_RESET_MILLIS){
    vehicle_run(0, 0);
    twist_last_millis = now_millis;
  }
}

void odometry_reset(struct ODOMETRY* odom){
  odom->is_reset_req = false;
  odom->last_theta = 0.0;
  odom->pos_x= 0.0;
  odom->pos_y = 0.0;
  odom->left_distance = 0.0;
  odom->right_distance = 0.0;
  odom->last_position_left = POSITION_UNKNOWN;
  odom->last_position_right = POSITION_UNKNOWN;
}

void odometry_udate(double speed_left, double speed_right, double position_left, double position_right){
  if(odom.is_reset_req){
    odometry_reset(&odom);
    return;
  }
  
  double angle_left, angle_right;
  angle_left = get_angle(position_left, odom.last_position_left);
  angle_right = get_angle(position_right, odom.last_position_right);
  // Serial.print(angle_left);
  // Serial.print(" ");
  // Serial.print(position_left);
  // Serial.print(" ");
  // Serial.println(odom.last_position_left);

  odom.last_position_left = position_left;
  odom.last_position_right = position_right;

  double left_velocity, right_velocity, left_distance, right_distance, distance, theta, d_x, d_y;
  geometry_msgs::Quaternion quat;

  left_velocity = 2.0 * M_PI *  (double)speed_left * WHEEL_TIRE_RADIUS / 60.0; //角速度[rpm]から並進速度[m/s]を出す 2*PI*R[m]*v[rpm]/60
  right_velocity = 2.0 * M_PI *  (double)speed_right * WHEEL_TIRE_RADIUS / 60.0; //角速度[rpm]から並進速度[m/s]を出す 2*PI*R[m]*v[rpm]/60
  left_distance = (double)angle_left / 180.0 * M_PI * WHEEL_TIRE_RADIUS; //距離[m]=回転角度[rad/s]*半径[m]
  right_distance = (double)angle_right / 180.0 * M_PI * WHEEL_TIRE_RADIUS; //距離[m]=回転角度[rad/s]*半径[m]
  distance = (left_distance + right_distance) / 2.0; //左右の車輪の移動距離の平均値
  theta = odom.last_theta + atan2(left_distance - right_distance, BASE_WIDTH); //<<<<<arctanは？　単位は？ 0度方向は？回転方向は？
  d_x = distance * cos(theta); // 軸の取り方は？
  d_y = distance * sin(theta);  // 軸の取り方は？
  quat = tf::createQuaternionFromYaw(theta); // クォータニオン

  odom.pos_x += d_x;
  odom.pos_y += d_y;  
  odom.last_theta = theta;
  odom.left_distance += left_distance;
  odom.right_distance += right_distance;
  
  // Serial.print(theta * 180 / M_PI);
  // Serial.print(" ");
  // Serial.print(odom.pos_x);
  // Serial.print(" ");
  // Serial.println(odom.pos_y);

  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "/odometry";
  odom_msg.child_frame_id = "base_footprint";

  /*位置情報を入れる*/
  odom_msg.pose.pose.position.x = odom.pos_x;
  odom_msg.pose.pose.position.y = odom.pos_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;

  odom_msg.twist.twist.linear.x = (left_velocity + right_velocity) / 2.0; // 並進速度[m/s]
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.z = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = (right_velocity - left_velocity) / BASE_WIDTH;// 角速度[rad/s]
}

double get_angle(double cur, double last){
  if(last == POSITION_UNKNOWN){
    return 0;
  }
  double angle = cur - last;

  if(angle >= 180.0){
    angle -= 360.0;
  }else if(angle < -180.0){
    angle += 360.0;
  }
  return angle;
}

void callBack_motor_control(const geometry_msgs::Twist& twist)
{
  if(operation_mode == AUTONOMOUS){
    /*モーター速度の計算*/
    float x, z;
    x = isnan(twist.linear.x) ? 0.0f : twist.linear.x;
    z = isnan(twist.angular.z) ? 0.0f : twist.angular.z;

    float left_velocity, right_velocity;
    left_velocity = (z * BASE_WIDTH - 2.0f * x) / (-2.0f); // vL[m/s] = (トレッド幅[m] × 角速度[rad/s] - 2 × 並進速度[m/s]) ÷ -2
    right_velocity = (z * BASE_WIDTH + 2.0f * x) / 2.0f; // vR[m/s] = (トレッド幅[m] × 角速度[rad/s] + 2 × 並進速度[m/s]) ÷ 2
    float left_rpm, right_rpm;
    left_rpm =  left_velocity * 60.0f / (2.0f * M_PI * WHEEL_TIRE_RADIUS); // ωL[rad/s] = 速度 × 60秒 ÷ 2πR
    right_rpm = right_velocity * 60.0f / (2.0f * M_PI * WHEEL_TIRE_RADIUS); // ωR[rad/s] = 速度 × 60秒 ÷ 2πR
    vehicle_run((int)left_rpm, (int)right_rpm);
    twist_last_millis = millis();
    char buf[100];
    sprintf(buf, "motor_ctrl %.2f, %.2f, %.2f, %.2f", twist.linear.x, twist.angular.z, left_rpm, right_rpm);
    // Serial.println(buf);
    nh.loginfo(buf);
  }else{
    nh.loginfo("operation_mode is not AUTONOMOUS...");
  }
}

void callBack_odometry_reset(const std_msgs::Empty& flag)
{
  odom.is_reset_req = true;
  nh.loginfo("Reset odometry!");
}
