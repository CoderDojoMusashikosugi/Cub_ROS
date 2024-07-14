#include <Arduino.h>
#include <M5Unified.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
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
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
sensor_msgs__msg__Imu msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_init_options_t init_options; // Humble
size_t domain_id = 1;

// for dynamixel valiable
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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    float linear_acceleration_x = 0.0;
    float linear_acceleration_y = 0.0;
    float linear_acceleration_z = 0.0;
    float angular_velocity_x = 0.0;
    float angular_velocity_y = 0.0;
    float angular_velocity_z = 0.0;

    M5.Imu.getGyroData(&angular_velocity_x, &angular_velocity_y, &angular_velocity_z);
    M5.Imu.getAccelData(&linear_acceleration_x, &linear_acceleration_y, &linear_acceleration_z);

    msg.linear_acceleration.x = linear_acceleration_x;
    msg.linear_acceleration.y = linear_acceleration_y;
    msg.linear_acceleration.z = linear_acceleration_z;
    msg.angular_velocity.x = angular_velocity_x;
    msg.angular_velocity.y = angular_velocity_y;
    msg.angular_velocity.z = angular_velocity_z;

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

// caluclate velocity for each dinamixel from twist value
float cub_d = 62.5;  // [mm] distance between center and wheel
float motor_vel_unit = 0.229;  //[rpm]
      
float diameter = 40; // [mm] diameter of wheel

void twist_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  RCUTILS_LOG_INFO("Received Twist message:");
  printf("Linear: x=%.2f, y=%.2f, z=%.2f\n", msg->linear.x, msg->linear.y, msg->linear.z);
  printf("Angular: x=%.2f, y=%.2f, z=%.2f\n", msg->angular.x, msg->angular.y, msg->angular.z);
  double r_vel_m = msg->linear.x + cub_d * msg->angular.z / 1000.0; // [m/s]
  double l_vel_m = msg->linear.x - cub_d * msg->angular.z / 1000.0; // [m/s]
  double r_vel_r = r_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  double l_vel_r = l_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  int32_t r_goal_vel = (int32_t)(r_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // right goal velocity
  int32_t l_goal_vel = (int32_t)(l_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // left goal velocity

  dxl.setGoalVelocity(DXL1_ID, r_goal_vel);
  delay(20);
  dxl.setGoalVelocity(DXL2_ID, l_goal_vel);
}

void remote_control(){
  Serial.println("ps5 loop");
  geometry_msgs__msg__Twist msg;
  float default_linear = 0.3;
  float default_angular = 2.0;
  while(1){
    if (ps5.isConnected()) {
      ps5.setLed(0, 255, 0);
      ps5.setFlashRate(100, 100);
      ps5.sendToController();
      if (ps5.L2()){
        if ((abs(ps5.LStickX()) < 5) && (abs(ps5.LStickY()) < 5)){
          msg.linear.x = 0;
          msg.angular.z = 0;
        } else {
          msg.linear.x = default_linear * (ps5.LStickY()) / 127.0;
          msg.angular.z = - default_angular * (ps5.LStickX()) / 127.0;
        }
        twist_callback(&msg);
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
  delay(1000);
  Serial.println("dxl ping");


  if (dxl.torqueOff(DXL1_ID)){
    leds[5] = CRGB::Red;
  } else {
    leds[5] = CRGB::Green;
  }
  FastLED.show();
  delay(1000);
  
  if (dxl.setOperatingMode(DXL1_ID, OP_VELOCITY)){
    leds[6] = CRGB::Red;
  } else {
    leds[6] = CRGB::Green;
  }
  FastLED.show();
  delay(200);
  
  if (dxl.torqueOn(DXL1_ID)) {
    leds[7] = CRGB::Red;
  } else {
    leds[7] = CRGB::Green;
  }
  FastLED.show();
  delay(200);

  if (dxl.torqueOff(DXL2_ID)){
    leds[10] = CRGB::Red;
  } else {
    leds[10] = CRGB::Green;
  }
  FastLED.show();
  delay(200);


  if (dxl.setOperatingMode(DXL2_ID, OP_VELOCITY)) {
    leds[11] = CRGB::Red;
  } else {
    leds[11] = CRGB::Green;
  }
  FastLED.show();
  delay(200);

  if (dxl.torqueOn(DXL2_ID)) {
    leds[12] = CRGB::Red;
  } else {
    leds[12] = CRGB::Green;
  }
  FastLED.show();
  delay(200);

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

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "pub_imu"));

  // create subscliber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "set_twist"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  leds[2] = CRGB::White;
  FastLED.show();

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &twist_callback, ON_NEW_DATA));

  leds[3] = CRGB::White;
  FastLED.show();
  delay(0.5);

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
