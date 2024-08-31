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
#include <geometry_msgs/msg/transform_stamped.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <Dynamixel2Arduino.h>

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
rcl_publisher_t tf_publisher;
rcl_publisher_t debug_publisher;
rcl_subscription_t subscriber;
std_msgs__msg__String debug_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__TransformStamped tf_stamp_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imu_timer;
rcl_timer_t odom_timer;
rcl_init_options_t init_options; // Humble
size_t domain_id = 1; // ros Domain ID

// for dynamixel valiable
/* DRIVE_MODE*/
const uint8_t ADDR_DRIVE_MODE = 10;
const uint8_t NORMAL_MODE = 0;
const uint8_t REVERSE_MODE = 1;
HardwareSerial& DXL_SERIAL  = Serial2; // 使うシリアル系統
Dynamixel2Arduino dxl;
const uint8_t RX_SERVO = 23;
const uint8_t TX_SERVO = 33;
const float DXL_PROTOCOL_VERSION = 2.0; //プロトコルのバージョン
const uint8_t DXL1_ID = 1;
const uint8_t DXL2_ID = 2;

// for LED valiable
CRGB leds[NUM_LEDS];

// for PS5 valiable
#include <ps5Controller.h>
#include <EspEasyTask.h>

EspEasyTask ps5task;

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
  Serial.println(message_buffer);

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

// caluclate velocity for each dinamixel from twist value
const float cub_d = 62.5;  // [mm] distance between center and wheel
const float motor_vel_unit = 0.229;  //[rpm]
const float diameter = 40; // [mm] diameter of wheel
const float ang_res = 0.088; // [deg/pluse] motor pluse resolution
int32_t l_motor_pos = 0;
int32_t r_motor_pos = 0;
// オドメトリ情報
double odom_x = 0.0;
double odom_y = 0.0;
double odom_theta = 0.0;


void update_odometry(int32_t delta_left, int32_t delta_right, double dt, double &x, double &y, double &theta, double &vx, double &vtheta) {
    // エンコーダのカウントをメートルに変換
    double d_left = delta_left * ang_res *  (M_PI / 180.0 * diameter / 1000.0);
    double d_right = delta_right * ang_res * (M_PI / 180.0 * diameter / 1000.0);
    debug_message("d left right (%lf, %lf)", d_left, d_right);

    // 距離の平均と回転角を計算
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / (cub_d / 1000.0); // rad?
    debug_message("d center theta (%lf, %lf)", d_center, d_theta);

    // ロボットの姿勢を更新
    x += d_center * cos(theta);
    y += d_center * sin(theta);
    theta += d_theta;
    debug_message("calculate x,y,theta=(%lf, %lf, %lf)", x, y, theta);
    // 速度を計算
    vx = d_center / dt;
    vtheta = d_theta / dt;
    debug_message("velocity vx, vtheta =(%lf, %lf)", vx, vtheta);
}

void odom_timer_callback(rcl_timer_t * odom_timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  static rcl_time_point_value_t last_time;
  rcl_time_point_value_t now_time;
  // 現在の時間を取得
  rcl_clock_t clock;
  RCCHECK(rcl_clock_init(RCL_ROS_TIME, &clock, &allocator));

  RCCHECK(rcl_clock_get_now(&clock, &now_time));

  if (odom_timer != NULL) {
    // 速度情報
    double vx = 0.0;
    double vy = 0.0;
    double vtheta = 0.0;
    // buffur clear
    while(DXL_SERIAL.available() > 0){
      DXL_SERIAL.read();
    }
    int32_t r_cur_pos = dxl.getPresentPosition(DXL1_ID);
    delay(5);
    int32_t l_cur_pos = dxl.getPresentPosition(DXL2_ID);
    delay(5);
    debug_message("got motor pos r = %ld l = %ld", r_cur_pos, l_cur_pos);

    // エンコーダのカウント差分を計算
    int32_t delta_left = l_cur_pos - l_motor_pos;
    int32_t delta_right = r_cur_pos - r_motor_pos;

    if (abs(delta_left) > 2000000000){
      if (delta_left > 0) {
        delta_left += 2147483647;
      } else{
        delta_left -= 2147483648;
      }
    }
    if (abs(delta_right) > 2000000000){
      if (delta_right > 0){
        delta_right += 2147483647;
      }else{
      delta_right -= 2147483648;
      }
    }
    debug_message("delta value = %ld, %ld", delta_left, delta_right);

    if ((abs(delta_left) > 20000) || (abs(delta_right) > 20000)) {
      return;
    }

    // 前の位置を更新
    l_motor_pos = l_cur_pos;
    r_motor_pos = r_cur_pos;

    double dt = (now_time - last_time) / 1e9;  // 秒に変換
    last_time = now_time;
    debug_message("dt = %lf", dt);

    // オドメトリを更新
    update_odometry(delta_left, delta_right, dt, odom_x, odom_y, odom_theta, vx, vtheta);

    // オドメトリメッセージを作成
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0);
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.angular.z = vtheta;

    debug_message("publish odometry");
    // オドメトリをパブリッシュ
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));


    // tf messageを作成
    // tf_stamp_msg.header.stamp.sec = dt; Todo UNIXタイムを入れる
    tf_stamp_msg.header.frame_id.data = "odom";
    tf_stamp_msg.header.frame_id.size = strlen(tf_stamp_msg.header.frame_id.data);
    tf_stamp_msg.header.frame_id.capacity = strlen(tf_stamp_msg.header.frame_id.data) + 1;
    tf_stamp_msg.child_frame_id.data = "base_link";
    tf_stamp_msg.child_frame_id.size = strlen(tf_stamp_msg.child_frame_id.data);
    tf_stamp_msg.child_frame_id.capacity = strlen(tf_stamp_msg.child_frame_id.data) + 1;

    tf_stamp_msg.transform.translation.x = odom_x;
    tf_stamp_msg.transform.translation.y = odom_y;
    tf_stamp_msg.transform.translation.z = 0;
    tf_stamp_msg.transform.rotation.x = 0;
    tf_stamp_msg.transform.rotation.y = 0;
    tf_stamp_msg.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    tf_stamp_msg.transform.rotation.w = odom_msg.pose.pose.orientation.w;

    debug_message("publish tf");
    RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_stamp_msg, NULL));
  }
}

void twist_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
  RCUTILS_LOG_INFO("Received Twist message:");
  printf("Linear: x=%.2f, y=%.2f, z=%.2f\n", twist_msg->linear.x, twist_msg->linear.y, twist_msg->linear.z);
  printf("Angular: x=%.2f, y=%.2f, z=%.2f\n", twist_msg->angular.x, twist_msg->angular.y, twist_msg->angular.z);
  double r_vel_m = twist_msg->linear.x + cub_d * twist_msg->angular.z / 1000.0; // [m/s]
  double l_vel_m = twist_msg->linear.x - cub_d * twist_msg->angular.z / 1000.0; // [m/s]
  double r_vel_r = r_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  double l_vel_r = l_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  int32_t r_goal_vel = (int32_t)(r_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // right goal velocity
  int32_t l_goal_vel = (int32_t)(l_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // left goal velocity

  dxl.setGoalVelocity(DXL1_ID, r_goal_vel);
  delay(5);
  dxl.setGoalVelocity(DXL2_ID, l_goal_vel);
}

void remote_control(){
  Serial.println("ps5 loop");
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
  
  auto cfg = M5.config();
  M5.begin(cfg);

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

  // initialize PS5
  connect_dualsense();
  ps5task.begin(remote_control, 2, 2);

  // initialize dynamixel
  DXL_SERIAL.begin(57600, SERIAL_8N1, RX_SERVO, TX_SERVO);
  dxl = Dynamixel2Arduino(DXL_SERIAL); //Dynamixel用ライブラリのインスタンス化
  dxl.begin(57600); // デフォルトのbaudrate. 必要に応じてサーボの設定にあわせる.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  leds[5] = CRGB::White;
  FastLED.show();
  
  Serial.println("dxl begin");
  // dxl.ping();
  if (dxl.ping(DXL1_ID) &&  dxl.ping(DXL2_ID)) {
    leds[5] = CRGB::Green;
  } else {
    leds[5] = CRGB::Red;
  }
  delay(5);
  Serial.println("dxl ping");


  if (dxl.torqueOff(DXL1_ID)){
    leds[5] = CRGB::Red;
  } else {
    leds[5] = CRGB::Green;
  }
  FastLED.show();
  delay(5);
  
  if (!dxl.write(DXL1_ID, ADDR_DRIVE_MODE, (uint8_t*)&REVERSE_MODE, 1)){
    leds[6] = CRGB::Red;
  } else {
    leds[6] = CRGB::Green;
  }
  delay(5);

  if (!dxl.setOperatingMode(DXL1_ID, OP_VELOCITY)){
    leds[7] = CRGB::Red;
  } else {
    leds[7] = CRGB::Green;
  }
  FastLED.show();
  delay(5);
  
  if (!dxl.torqueOn(DXL1_ID)) {
    leds[8] = CRGB::Red;
  } else {
    leds[8] = CRGB::Green;
  }
  FastLED.show();
  delay(5);

  if (!dxl.torqueOff(DXL2_ID)){
    leds[10] = CRGB::Red;
  } else {
    leds[10] = CRGB::Green;
  }
  FastLED.show();
  delay(5);
  
  if (!dxl.write(DXL2_ID, ADDR_DRIVE_MODE, (uint8_t*)&NORMAL_MODE, 1)){
    leds[11] = CRGB::Red;
  } else {
    leds[11] = CRGB::Green;
  }
  delay(5);

  if (!dxl.setOperatingMode(DXL2_ID, OP_VELOCITY)) {
    leds[12] = CRGB::Red;
  } else {
    leds[12] = CRGB::Green;
  }
  FastLED.show();
  delay(5);

  if (!dxl.torqueOn(DXL2_ID)) {
    leds[13] = CRGB::Red;
  } else {
    leds[13] = CRGB::Green;
  }
  delay(5);

  // buffur clear
  while(DXL_SERIAL.available() > 0){
    DXL_SERIAL.read();
  }

  r_motor_pos = dxl.getPresentPosition(DXL1_ID);
  delay(5);
  l_motor_pos = dxl.getPresentPosition(DXL2_ID);
  FastLED.show();
  delay(5);

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

  // create tf_publisher
  RCCHECK(rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped),
    "/tf"));

  // create subscliber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "set_twist"));

  // create imu_timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &imu_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    imu_timer_callback));
  
  RCCHECK(rclc_timer_init_default(
    &odom_timer,
    &support,
    RCL_MS_TO_NS(500),
    odom_timer_callback));

  leds[2] = CRGB::White;
  FastLED.show();

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));
  
  debug_message("initial motor val l,r = (%ld, %ld)", r_motor_pos, l_motor_pos);

  leds[3] = CRGB::White;
  FastLED.show();
  delay(10);
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
