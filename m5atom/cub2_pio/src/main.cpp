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

#include <DDT_Motor_M15M06.h>

// for control LED
#include <FastLED.h>
#define LED_DATA_PIN 27
#define LANE_LED_DATA_PIN 26
#define NUM_LEDS 25
#define NUM_LANE_LEDS 24

#define EMERGENCY_MONITOR (GPIO_NUM_19)

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
geometry_msgs__msg__Twist send_twist_msg;
geometry_msgs__msg__Twist remote_twist_msg;
geometry_msgs__msg__Twist subscribe_twist_msg;
std_msgs__msg__Int32MultiArray wheel_positions_msg;
unsigned long prev_msg_time = 0;

rclc_executor_t executor;
rclc_executor_t sub_executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imu_timer;
rcl_timer_t wh_pos_timer;
rcl_timer_t motor_controll_timer;
rcl_init_options_t init_options; // Humble
size_t domain_id = 0;


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

uint16_t right_wheel_position1 = 0;
uint16_t right_wheel_position2 = 0;
uint16_t left_wheel_position1 = 0;
uint16_t left_wheel_position2 = 0;

int last_num_sign[2] = {0,0};
const int16_t SPEED_MAX = 115;  //DDSM115 115rpm = max11
const int16_t SPEED_MIN = -115;

// Mutexの宣言
SemaphoreHandle_t mutex;
// タイマーのハンドル
TimerHandle_t motorControlTimer;

enum uros_states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} uros_state;

enum robo_modes{
  EMERGENCY,
  IDLE,
  REMOTE_CTRL,
  AUTONOMOUS
} robo_mode;


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
CRGB lane_led[NUM_LANE_LEDS];

// for PS5 valiable
#include <ps5Controller.h>
#include <EspEasyTask.h>

EspEasyTask subsc_task;

void connect_dualsense(){
  ps5.begin("4C:B9:9B:64:76:1A"); //replace with your MAC address
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

void subsc_loop() {
  while(1) {
    if (uros_state == AGENT_CONNECTED) {
      RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(20)));  // threadセーフではないことに注意
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
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
    // debug_message("i = %d send speed %d receiver id %d temp %d err %d", i, Speed[i], Receiv.ID, Receiv.Temp, Receiv.ErrCode);
    vTaskDelay(5 / portTICK_PERIOD_MS);//1回の通信ごとに5msのWaitが必要（RS485の半二重通信の問題と思われる）
    
    switch (i)
    {
    case 0:
      left_wheel_position2 = Receiv.Position;
      break;
    case 1:
      right_wheel_position1 = Receiv.Position;
      break;
    case 2:
      left_wheel_position1 = Receiv.Position;
      break;
    case 3:
      right_wheel_position2 = Receiv.Position;
      break;
    default:
      break;
    }
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
  motor_exec();
} 

void emergency_stop() {
    motor_brake();
    robo_mode = EMERGENCY;
    leds[4] = CRGB::Red;
    fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Red);
    FastLED.show();
}


// caluclate velocity for each dinamixel from twist value
const float cub_d = 110;  // [mm] distance between center and wheel
float motor_vel_unit = 1;  //[rpm]
const float diameter = 150; // [mm] diameter of wheel
uint16_t l_motor_pos = 0;
uint16_t r_motor_pos = 0;

geometry_msgs__msg__Twist vehicle_stop_msg() {
  geometry_msgs__msg__Twist twist_msg;
  twist_msg.linear.x = 0;
  twist_msg.linear.y = 0;
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = 0;
  return twist_msg;
}

void wh_pos_timer_callback(rcl_timer_t * wh_pos_timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  if (wh_pos_timer != NULL) {
    
    debug_message("rightpos left pos %d, %d, %d, %d", right_wheel_position1, right_wheel_position2, left_wheel_position1, left_wheel_position2);

    // メッセージデータの設定
    wheel_positions_msg.data.data[0] = static_cast<int32_t>(left_wheel_position1);
    wheel_positions_msg.data.data[1] = static_cast<int32_t>(right_wheel_position1);
    wheel_positions_msg.data.data[2] = static_cast<int32_t>(left_wheel_position2);
    wheel_positions_msg.data.data[3] = static_cast<int32_t>(right_wheel_position2);

    // メッセージのパブリッシュ
    RCCHECK(rcl_publish(&wh_pos_publisher, &wheel_positions_msg, NULL));
  }
}

void twist_callback(const void * msgin)
{
  prev_msg_time = millis();
}

void motor_controll_callback(TimerHandle_t xTimer)
{
  switch (robo_mode)
  {
  case IDLE:
    motor_stop();
    send_twist_msg = vehicle_stop_msg();
    break;
  case REMOTE_CTRL:
    send_twist_msg = remote_twist_msg;
    break;
  case AUTONOMOUS:
    if (millis() - prev_msg_time > 3000) {
      robo_mode = IDLE;
      motor_stop();
      subscribe_twist_msg = vehicle_stop_msg();
    }
    send_twist_msg = subscribe_twist_msg;
    break;
  case EMERGENCY:
    motor_brake();
    remote_twist_msg = vehicle_stop_msg();
    subscribe_twist_msg = vehicle_stop_msg();
    send_twist_msg = vehicle_stop_msg();
    break;
  default:
    motor_stop();
    send_twist_msg = vehicle_stop_msg();
    break;
  }

  double r_vel_m = send_twist_msg.linear.x + cub_d * send_twist_msg.angular.z / 1000.0; // [m/s]
  double l_vel_m = send_twist_msg.linear.x - cub_d * send_twist_msg.angular.z / 1000.0; // [m/s]
  double r_vel_r = r_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  double l_vel_r = l_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  int r_goal_vel = (int)(r_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // right goal velocity[rpm] # TODO: check whether need to add dvidid by motor_velocity_unit
  int l_goal_vel = (int)(l_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // left goal velocity[rpm]

  vehicle_run(r_goal_vel, l_goal_vel);
}

void remote_control(){
  float default_linear = 1.2;
  float default_angular = 2.0;
  if (ps5.isConnected()) {
    if (robo_mode == IDLE) robo_mode = REMOTE_CTRL;
    leds[2] = CRGB::Green;

    ps5.setLed(0, 255, 0);
    ps5.setFlashRate(100, 100);
    ps5.sendToController();
    if (ps5.Cross()) {
      remote_twist_msg = vehicle_stop_msg();
      robo_mode = EMERGENCY;
    }

    if (ps5.L2()) {
      int val_LStickX = ps5.LStickX();
      int val_LStickY = ps5.LStickY();
      if (abs(val_LStickX) < 16) val_LStickX = 0;
      if (abs(val_LStickY) < 16) val_LStickY = 0;
      remote_twist_msg.linear.x = default_linear * (val_LStickY) / 127.0;
      remote_twist_msg.angular.z = - default_angular * (val_LStickX) / 127.0;
    } else if (ps5.Share() && ps5.Options()) {
      ESP.restart();
    } else if (ps5.Options()){ //自律走行モード
        robo_mode = AUTONOMOUS;
      }
      else if (ps5.Share()){
        robo_mode = REMOTE_CTRL; //遠隔操作モード
      }
    else {
      remote_twist_msg = vehicle_stop_msg();
    }
  } else {
    leds[2] = CRGB::Red;
    if (robo_mode == EMERGENCY) return;
    
    if (uros_state == AGENT_CONNECTED) robo_mode = AUTONOMOUS;
    else robo_mode = IDLE;
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
  // RCCHECK(rclc_publisher_init_default(
  //   &imu_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  //   "pub_imu"));

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
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &imu_timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   imu_timer_callback));
  
  // create wh_pos_timer
  RCCHECK(rclc_timer_init_default(
    &wh_pos_timer,
    &support,
    RCL_MS_TO_NS(100),
    wh_pos_timer_callback));
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_init(&sub_executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &wh_pos_timer));

  RCCHECK(rclc_executor_add_subscription(&sub_executor, &subscriber, &subscribe_twist_msg, &twist_callback, ON_NEW_DATA));
  
  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_publisher_fini(&debug_publisher, &node));
  // RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
  RCCHECK(rcl_publisher_fini(&wh_pos_publisher, &node));
  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  // RCCHECK(rcl_timer_fini(&imu_timer));
  RCCHECK(rcl_timer_fini(&wh_pos_timer));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rclc_executor_fini(&sub_executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));
  RCCHECK(rcl_init_options_fini(&init_options));
}

void setup() {
  robo_mode = IDLE;
  pinMode(EMERGENCY_MONITOR, INPUT); 
  // attachInterrupt(EMERGENCY_MONITOR, emergency_stop, RISING);
  // gpio_pulldown_dis(EMERGENCY_MONITOR);
  
  // auto cfg = M5.config();
  // M5.begin(cfg);
  // auto motor_handler = MotorHandler(33, 23); // Cub2 ATOM(33, 23) Cub1 RX,TX ATOM(32, 26) DDSM210 ATOM S3(2,1)

  // Configure serial transport
  Serial.begin(1500000);
  set_microros_serial_transports(Serial);
  delay(2000);

  // initialize LED
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LANE_LED_DATA_PIN>(lane_led, NUM_LANE_LEDS);
  FastLED.setBrightness(200);
  leds[0] = CRGB::White;
  fill_solid(lane_led, NUM_LANE_LEDS, CRGB( 255, 255, 255));
  lane_led[3] = CRGB::Aqua;
  FastLED.show();
  delay(10);

  for(int i=1;i<=4;i++){
    motor_handler.Control_Motor(0, i, Acce, Brake_P, &Receiv); //4つのモーター
  }

  // initialize PS5
  connect_dualsense();

  // メッセージの初期化
  std_msgs__msg__Int32MultiArray__init(&wheel_positions_msg);
  wheel_positions_msg.data.capacity = 4;
  wheel_positions_msg.data.size = 4;
  wheel_positions_msg.data.data = (int32_t*) malloc(4 * sizeof(int32_t));
  
  // タイマーを作成（40ms周期）
  motorControlTimer = xTimerCreate("MotorControlTimer",     // タイマーの名前
                                   pdMS_TO_TICKS(40),       // タイマー周期 (20ms)
                                   pdTRUE,                  // 自動リロード
                                   (void *)0,               // タイマーID
                                   motor_controll_callback  // コールバック関数
                                   );
  if (motorControlTimer == NULL) {
    leds[1] = CRGB::Red;
    FastLED.show();
  } else {
    leds[1] = CRGB::Blue;
    FastLED.show();
    // タイマーのスタート
    if (xTimerStart(motorControlTimer, 0) != pdPASS) {
      leds[1] = CRGB::Purple;
      FastLED.show();
    } else {
      leds[1] = CRGB::Green;
      FastLED.show();
    }
  }
}

void loop() {
  if (digitalRead(EMERGENCY_MONITOR) == HIGH) {
    motor_brake();
    robo_mode = EMERGENCY;
    leds[4] = CRGB::Red;
    fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Red);
    FastLED.show();
    return;
  } else {
    if (robo_mode == EMERGENCY) robo_mode = IDLE;
  }

  EXECUTE_EVERY_N_MS(50, remote_control());

  if (robo_mode == REMOTE_CTRL) {
    leds[4] = CRGB::Yellow;
    fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Yellow);
  } else if (robo_mode == AUTONOMOUS) {
    leds[4] = CRGB::Green;
    fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Green);
  } else if (robo_mode == IDLE) {
    leds[4] = CRGB::Blue;
    fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Blue);
  }

  switch (uros_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
                         uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                     ? AGENT_AVAILABLE
                                     : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      uros_state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (uros_state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
                         uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                     ? AGENT_CONNECTED
                                     : AGENT_DISCONNECTED;);
      if (uros_state == AGENT_CONNECTED) {
        RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(20)));  // threadセーフではないことに注意
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));  // threadセーフではないことに注意
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      uros_state = WAITING_AGENT;
      break;
    default:
      break;
  }

  EVERY_N_MILLISECONDS(200) {
    if (uros_state == AGENT_CONNECTED) {
      leds[0] = CRGB::Green;
    } else {
      leds[0] = CRGB::Red;
    }
    FastLED.show();
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}
