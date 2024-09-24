// Reference Code https://www.hackster.io/amal-shaji/differential-drive-robot-using-ros2-and-esp32-aae289

#include <Arduino.h>
#include <M5Unified.h>
#include <micro_ros_platformio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <Dynamixel2Arduino.h>

// for control LED
#include <FastLED.h>
#define LED_DATA_PIN 27
#define NUM_LEDS 25

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define DEBUG

// micro-ros valiable
rcl_publisher_t imu_publisher;
rcl_publisher_t wh_pos_publisher;
rcl_publisher_t debug_publisher;
rcl_subscription_t subscriber;
std_msgs__msg__String debug_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32MultiArray wheel_positions_msg;
unsigned long prev_cmd_time = 0;

rclc_executor_t executor;
rclc_executor_t sub_executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imu_timer;
rcl_timer_t wh_pos_timer;
rcl_timer_t motor_controll_timer;
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

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define EXECUTE_EVERY_N_MS(MS, X)      \
  do {                                 \
    static volatile int64_t init = -1; \
    if (init == -1) {                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS) {    \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

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

void debug_message(const char* format, ...);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop(rcl_ret_t temp_rc) {
  leds[24] = CRGB::Red;
  FastLED.show();
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  
  rcl_reset_error();
  leds[24] = CRGB::Black;
  FastLED.show();
}

// デバッグメッセージを出力する関数
char message_buffer[255];
void debug_message(const char* format, ...) {
  #ifdef DEBUG
  va_list args;
  va_start(args, format);
  vsnprintf(message_buffer, sizeof(message_buffer), format, args);
  va_end(args);

  // ROSトピックにパブリッシュ
  debug_msg.data.data = message_buffer;
  debug_msg.data.size = strlen(message_buffer);
  debug_msg.data.capacity = sizeof(message_buffer);
  RCCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
  #endif
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

void wh_pos_timer_callback(rcl_timer_t * wh_pos_timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  if (wh_pos_timer != NULL) {
    // buffur clear
    while(DXL_SERIAL.available() > 0){
      DXL_SERIAL.read();
    }
    int32_t right_wheel_position = dxl.getPresentPosition(DXL1_ID);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    int32_t left_wheel_position = dxl.getPresentPosition(DXL2_ID);
    vTaskDelay(5 / portTICK_PERIOD_MS);

    // メッセージデータの設定
    wheel_positions_msg.data.data[0] = left_wheel_position;
    wheel_positions_msg.data.data[1] = right_wheel_position;

    // メッセージのパブリッシュ
    RCCHECK(rcl_publish(&wh_pos_publisher, &wheel_positions_msg, NULL));
  }
}

void twist_callback(const void * msgin)
{
  prev_cmd_time = millis();
}

void motor_controll_callback(rcl_timer_t * motor_callback_timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  double r_vel_m = twist_msg.linear.x + cub_d * twist_msg.angular.z / 1000.0; // [m/s]
  double l_vel_m = twist_msg.linear.x - cub_d * twist_msg.angular.z / 1000.0; // [m/s]
  double r_vel_r = r_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  double l_vel_r = l_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  int32_t r_goal_vel = (int32_t)(r_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // right goal velocity
  int32_t l_goal_vel = (int32_t)(l_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // left goal velocity

  dxl.setGoalVelocity(DXL1_ID, r_goal_vel);
  vTaskDelay(5 / portTICK_PERIOD_MS);
  dxl.setGoalVelocity(DXL2_ID, l_goal_vel);
  vTaskDelay(5 / portTICK_PERIOD_MS);
}

void remote_control(){
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
        rcl_timer_t* temp_timer = nullptr;
        motor_controll_callback(temp_timer, 0);
      } else if (ps5.Share() && ps5.Options()) {
        ESP.restart();
      }
    } else {
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

bool create_entities() {
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

  // create wh_pos_publisher
  RCCHECK(rclc_publisher_init_default(
    &wh_pos_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "wheel_positions"));


  // create subscliber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
    
  // create imu_timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &imu_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    imu_timer_callback));
  
  // create wh_pos_timer
  RCCHECK(rclc_timer_init_default(
    &wh_pos_timer,
    &support,
    RCL_MS_TO_NS(100),
    wh_pos_timer_callback));
  
  // create motor controll timer
  RCCHECK(rclc_timer_init_default(
    &motor_controll_timer,
    &support,
    RCL_MS_TO_NS(100),
    motor_controll_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_init(&sub_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &wh_pos_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &motor_controll_timer));

  RCCHECK(rclc_executor_add_subscription(&sub_executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));
  
  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  // free(wheel_positions_msg.data.data);
  // std_msgs__msg__Int32MultiArray__destroy(&wheel_positions_msg);

  rcl_publisher_fini(&debug_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&wh_pos_publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_timer_fini(&imu_timer);
  rcl_timer_fini(&wh_pos_timer);
  rcl_timer_fini(&motor_controll_timer);
  rclc_executor_fini(&executor);
  rclc_executor_fini(&sub_executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  rcl_init_options_fini(&init_options);
}



void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  // Configure serial transport
  Serial.begin(1500000);
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
  
  // メッセージの初期化
  std_msgs__msg__Int32MultiArray__init(&wheel_positions_msg);
  wheel_positions_msg.data.capacity = 2;
  wheel_positions_msg.data.size = 2;
  wheel_positions_msg.data.data = (int32_t*) malloc(2 * sizeof(int32_t));

  // initialize dynamixel
  DXL_SERIAL.begin(57600, SERIAL_8N1, RX_SERVO, TX_SERVO);
  dxl = Dynamixel2Arduino(DXL_SERIAL); //Dynamixel用ライブラリのインスタンス化
  dxl.begin(57600); // デフォルトのbaudrate. 必要に応じてサーボの設定にあわせる.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  leds[5] = CRGB::White;
  FastLED.show();
  
  if (dxl.ping(DXL1_ID) &&  dxl.ping(DXL2_ID)) {
    leds[5] = CRGB::Green;
  } else {
    leds[5] = CRGB::Red;
  }
  delay(5);


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

  delay(10);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                     ? AGENT_AVAILABLE
                                     : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                     ? AGENT_CONNECTED
                                     : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  // threadセーフではないことに注意
        RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(100)));  // threadセーフではないことに注意
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    leds[0] = CRGB::Green;
    FastLED.show();
  } else {
    leds[0] = CRGB::Red;
    FastLED.show();
  }
}
