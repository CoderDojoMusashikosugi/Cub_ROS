#include <Arduino.h>
#include <M5Unified.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <DDT_Motor_M15M06.h>

// for control LED
#include <FastLED.h>
#define LED_DATA_PIN 27
#define NUM_LEDS 25

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

// micro-ros valiable
rcl_publisher_t imu_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t debug_publisher;
rcl_subscription_t subscriber;
std_msgs__msg__String debug_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imu_timer;
rcl_timer_t odom_timer;
rcl_init_options_t init_options; // Humble
size_t domain_id = 1;


// for cub motor valiable
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

int last_num_sign[2] = {0,0};
const int16_t SPEED_MAX = 115;  //DDSM115 115rpm = max11
const int16_t SPEED_MIN = -115;

// for LED valiable
CRGB leds[NUM_LEDS];

// for PS5 valiable
#include <ps5Controller.h>
// #include <EspEasyTask.h>

// EspEasyTask ps5task;

void connect_dualsense(){
  ps5.begin("e8:47:3a:34:44:a6"); //replace with your MAC address
  esp_log_level_set("ps5_L2CAP", ESP_LOG_VERBOSE);
  esp_log_level_set("ps5_SPP", ESP_LOG_VERBOSE);  
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  printf("Error at line %d: %s\n", __LINE__, rcl_get_error_string().str);
  while(1) {
    delay(100);
  }
}

char message_buffer[100];
// デバッグメッセージを出力する関数
void debug_message(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(message_buffer, sizeof(message_buffer), format, args);
  va_end(args);

  // シリアルモニタに出力
  // Serial.println(message_buffer);

  // ROSトピックにパブリッシュ
  debug_msg.data.data = message_buffer;
  debug_msg.data.size = strlen(message_buffer);
  debug_msg.data.capacity = sizeof(message_buffer);
  rcl_publish(&debug_publisher, &debug_msg, NULL);
}

void imu_timer_callback(rcl_timer_t * imu_timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (imu_timer != NULL) {
    float linear_acceleration_x = 0.0;
    float linear_acceleration_y = 0.0;
    float linear_acceleration_z = 0.0;
    float angular_velocity_x = 0.0;
    float angular_velocity_y = 0.0;
    float angular_velocity_z = 0.0;

    M5.Imu.getGyroData(&angular_velocity_x, &angular_velocity_y, &angular_velocity_z);
    M5.Imu.getAccelData(&linear_acceleration_x, &linear_acceleration_y, &linear_acceleration_z);

    imu_msg.linear_acceleration.x = linear_acceleration_x;
    imu_msg.linear_acceleration.y = linear_acceleration_y;
    imu_msg.linear_acceleration.z = linear_acceleration_z;
    imu_msg.angular_velocity.x = angular_velocity_x;
    imu_msg.angular_velocity.y = angular_velocity_y;
    imu_msg.angular_velocity.z = angular_velocity_z;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}


void motor_exec(){
  for(int i=0;i<4;i++){
    motor_handler.Control_Motor(Speed[i], i+1, Acce, brake, &Receiv);//スピード0：モーター停止
    debug_message("i = %d send speed %d receiver id %d temp %d err %d", i, Speed[i], Receiv.ID, Receiv.Temp, Receiv.ErrCode);
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
  Speed[0]=Speed[2]=-right;
  Speed[1]=Speed[3]=left;
  brake = Brake_Disable;
  debug_message("vehicle run value %d %d", right, left);
  motor_exec();
} 

// caluclate velocity for each dinamixel from twist value
const float cub_d = 800;  // [mm] distance between center and wheel
float motor_vel_unit = 0.001;  //[rpm]
const float diameter = 550; // [mm] diameter of wheel
int32_t l_motor_pos = 0;
int32_t r_motor_pos = 0;
// オドメトリ情報
double odom_x = 0.0;
double odom_y = 0.0;
double odom_theta = 0.0;


void update_odometry(int32_t left_position, int32_t right_position, double dt, double &x, double &y, double &theta, double &vx, double &vtheta) {
    // エンコーダのカウント差分を計算
    int32_t delta_left = left_position - l_motor_pos;
    int32_t delta_right = right_position - r_motor_pos;

   if (delta_left < 2*10^9){
      delta_left += 2147483647;
    } else if (delta_left > 2*10^9){
      delta_left -= 2147483648;
    }
    if (delta_right < 2*10^9){
      delta_right += 2147483647;
    } else if (delta_right > 2*10^9){
      delta_right -= 2147483648;
    }

    // 前の位置を更新
    l_motor_pos = left_position;
    r_motor_pos = right_position;

    // エンコーダのカウントをメートルに変換
    double d_left = delta_left * (2.0 * M_PI * diameter);
    double d_right = delta_right * (2.0 * M_PI * diameter);

    // 距離の平均と回転角を計算
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / cub_d;

    // ロボットの姿勢を更新
    x += d_center * cos(theta);
    y += d_center * sin(theta);
    theta += d_theta;

    // 速度を計算
    vx = d_center / dt;
    vtheta = d_theta / dt;
}

void odom_timer_callback(rcl_timer_t * odom_timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  static rcl_time_point_value_t last_time;
  rcl_time_point_value_t now_time;
  // 現在の時間を取得
  rcl_clock_t clock;
  RCCHECK(rcl_clock_init(RCL_ROS_TIME, &clock, &allocator));

  RCCHECK(rcl_clock_get_now(&clock, &now_time));
  double dt = (now_time - last_time) / 1e9;  // 秒に変換
  last_time = now_time;

  if (odom_timer != NULL) {

    // 前のモータ位置情報
    int32_t l_motor_pos = 0;
    int32_t r_motor_pos = 0;

    // 速度情報
    double vx = 0.0;
    double vy = 0.0;
    double vtheta = 0.0;
    
    // int32_t r_cur_pos = dxl.getPresentPosition(DXL1_ID);
    // int32_t l_cur_pos = dxl.getPresentPosition(DXL2_ID);

    // オドメトリを更新
    // update_odometry(l_cur_pos, r_cur_pos, dt, odom_x, odom_y, odom_theta, vx, vtheta);

    // オドメトリメッセージを作成
    nav_msgs__msg__Odometry odom_msg;
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0);
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.angular.z = vtheta;

    // オドメトリをパブリッシュ
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
  }
}

void twist_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
  debug_message("Received Twist message:");
  debug_message("Linear: x=%.2f, y=%.2f, z=%.2f", twist_msg->linear.x, twist_msg->linear.y, twist_msg->linear.z);
  debug_message("Angular: x=%.2f, y=%.2f, z=%.2f", twist_msg->angular.x, twist_msg->angular.y, twist_msg->angular.z);
  double r_vel_m = twist_msg->linear.x + cub_d * twist_msg->angular.z / 1000.0; // [m/s]
  double l_vel_m = twist_msg->linear.x - cub_d * twist_msg->angular.z / 1000.0; // [m/s]
  double r_vel_r = r_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  double l_vel_r = l_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  int r_goal_vel = (int)(r_vel_r / (2 * M_PI) * 60.0); // right goal velocity[rpm]
  int l_goal_vel = (int)(l_vel_r / (2 * M_PI) * 60.0); // left goal velocity[rpm]

  debug_message("goal velocity (r,l)=(%d, %d)", r_goal_vel, l_goal_vel);
  vehicle_run(r_goal_vel, l_goal_vel);
  // dxl.setGoalVelocity(DXL1_ID, r_goal_vel);
  // delay(20);
  // dxl.setGoalVelocity(DXL2_ID, l_goal_vel);
}

void remote_control(){
  // Serial.println("ps5 loop");
  geometry_msgs__msg__Twist twist_msg;
  float default_linear = 0.3;
  float default_angular = 2.0;
  while(1){
    if (ps5.isConnected()) {
      ps5.setLed(0, 255, 0);
      ps5.setFlashRate(100, 100);
      ps5.sendToController();
      if (ps5.L2()){
        if ((abs(ps5.LStickX()) < 5) && (abs(ps5.LStickY()) < 5)){
          twist_msg.linear.x = 0;
          twist_msg.angular.z = 0;
        } else {
          twist_msg.linear.x = default_linear * (ps5.LStickY()) / 127.0;
          twist_msg.angular.z = - default_angular * (ps5.LStickX()) / 127.0;
        }
        twist_callback(&twist_msg);
      } else if (ps5.Share() && ps5.Options()) {
        ESP.restart();
      }
    } else {
    }
    delay(50);
  }
}

void setup() {
  
  // auto cfg = M5.config();
  // M5.begin(cfg);
  // auto motor_handler = MotorHandler(33, 23); // Cub2 ATOM(33, 23) Cub1 RX,TX ATOM(32, 26) DDSM210 ATOM S3(2,1)

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // initialize LED
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(20);
  leds[0] = CRGB::White;
  FastLED.show();
  delay(10);

  for(int i=1;i<=4;i++){
    motor_handler.Control_Motor(0, i, Acce, Brake_P, &Receiv); //4つのモーター
  }

  // initialize PS5
  connect_dualsense();
  // ps5task.begin(remote_control, 2, 2);

  leds[5] = CRGB::White;
  FastLED.show();
  
  // initialize micro-ros
  allocator = rcl_get_default_allocator();

  /* Humble */
  // create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator)); 
  // Set ROS domain id
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));
  // Setup support structure.
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  leds[1] = CRGB::White;
  FastLED.show();

  // create debug_publisher
  RCCHECK(rclc_publisher_init_default(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "mros_debug_topic"));
    
  // create imu_publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "pub_imu"));

  // create odom_publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // create subscliber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "set_twist"));

  // create imu_timer,
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &imu_timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   imu_timer_callback));
  
  // RCCHECK(rclc_timer_init_default(
  //   &odom_timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   odom_timer_callback));

  leds[2] = CRGB::White;
  FastLED.show();

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  // RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));

  leds[3] = CRGB::White;
  FastLED.show();
  delay(10);
}


void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}