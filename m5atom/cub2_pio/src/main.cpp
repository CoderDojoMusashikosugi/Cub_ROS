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

#ifdef CUB_TARGET_CUB2
#include <DDT_Motor_M15M06.h>
#elif defined(CUB_TARGET_MCUB)
#include <Dynamixel2Arduino.h>
#endif

// for control LED
#include <FastLED.h>
#define LED_DATA_PIN 27
#define NUM_LEDS 25
#define LANE_LED_DATA_PIN 26
#define NUM_LANE_LEDS 24

#define EMERGENCY_MONITOR (GPIO_NUM_19)

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
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imu_timer;
rcl_init_options_t init_options; // Humble

#ifdef CUB_TARGET_CUB2
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
Receiver Receiv[4]; // 0:rear left, 1:front right, 2:front left, 3:rear right
// M5Stackのモジュールによって対応するRX,TXのピン番号が違うためM5製品とRS485モジュールに対応させてください
auto motor_handler = MotorHandler(33, 23); // Cub2 ATOM(33, 23) Cub1 RX,TX ATOM(32, 26) DDSM210 ATOM S3(2,1)

uint16_t front_right_wheel_position = 0;
uint16_t rear_right_wheel_position = 0;
uint16_t front_left_wheel_position = 0;
uint16_t rear_left_wheel_position = 0;

int last_num_sign[2] = {0,0};
const int16_t SPEED_MAX = 115;  //DDSM115 115rpm = max11
const int16_t SPEED_MIN = -115;

#elif defined(CUB_TARGET_MCUB)
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
#endif

// Mutexの宣言
volatile SemaphoreHandle_t mutex;
// タイマーのハンドル
TimerHandle_t motorControlTimer;

enum uros_states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} uros_state;

volatile enum robo_modes{
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
// #include <EspEasyTask.h>

void connect_dualsense(){
  ps5.begin("4C:B9:9B:64:76:1A"); //replace with your MAC address
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

void IRAM_ATTR set_emergency(){
  robo_mode = EMERGENCY;
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

#ifdef CUB_TARGET_CUB2
void motor_exec(){
  if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100))) {
    for(int i=0;i<4;i++){
      motor_handler.Control_Motor(Speed[i], i+1, Acce, brake, &Receiv[i]);//スピード0：モーター停止
      // debug_message("i = %d send speed %d receiver id %d temp %d err %d", i, Speed[i], Receiv.ID, Receiv.Temp, Receiv.ErrCode);
      vTaskDelay(5 / portTICK_PERIOD_MS);//1回の通信ごとに5msのWaitが必要（RS485の半二重通信の問題と思われる） 
    }
    xSemaphoreGive(mutex);
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

unsigned int vehicle_stop_exe_count = 0;
void vehicle_run(int right, int left){
  Speed[0]=Speed[2]=-right;
  Speed[1]=Speed[3]=left;
  if(vehicle_stop_exe_count > 10) {
    brake = Brake_Enable;
  } else {
    brake = Brake_Disable;
  }
  motor_exec();
  if((right == 0) && (left == 0)) {
    ++vehicle_stop_exe_count;
  } else {
    vehicle_stop_exe_count = 0;
  }
} 

int16_t get_max_motor_speed()
{
  int16_t max_speed = 0;
  for(int i=0;i<4;i++){
    if (max_speed < abs(Receiv[i].BSpeed)) max_speed = abs(Receiv[i].BSpeed);
  }
  return max_speed;
}

void emergency_stop() {
  robo_mode = EMERGENCY;
  leds[4] = CRGB::Red;
  fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Red);
  FastLED.show();
  // モータ回転状態でbrakeを入れると、大電流が流れるため安全に停止する
  while (get_max_motor_speed() > 10) {
    vehicle_run(0, 0);  
  }
  motor_brake();
}

#elif defined(CUB_TARGET_MCUB)
void initialize_dynamixel() {
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
}
#endif

// caluclate velocity for each dinamixel from twist value
#ifdef CUB_TARGET_CUB2
const float cub_d = 110;  // [mm] distance between center and wheel
float motor_vel_unit = 1;  //[rpm]
const float diameter = 150; // [mm] diameter of wheel
uint16_t l_motor_pos = 0;
uint16_t r_motor_pos = 0;
#elif defined(CUB_TARGET_MCUB)
const float cub_d = 62.5;  // [mm] distance between center and wheel
const float motor_vel_unit = 0.229;  //[rpm]
const float diameter = 40; // [mm] diameter of wheel
const float ang_res = 0.088; // [deg/pluse] motor pluse resolution
int32_t l_motor_pos = 0;
int32_t r_motor_pos = 0;
#endif

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

void wh_pos_timer_callback() {
  
    // メッセージデータの設定
#ifdef CUB_TARGET_CUB2
  for(int i=0;i<4;i++) {
    switch (i)
    {
    case 0:
      rear_left_wheel_position = Receiv[i].Position;
      break;
    case 1:
      front_left_wheel_position = Receiv[i].Position;
      break;
    case 2:
      front_left_wheel_position = Receiv[i].Position;
      break;
    case 3:
      rear_left_wheel_position = Receiv[i].Position;
      break;
    default:
      break;
    }
  }
  wheel_positions_msg.data.data[0] = static_cast<int32_t>(front_left_wheel_position);
  wheel_positions_msg.data.data[1] = static_cast<int32_t>(front_right_wheel_position);
  wheel_positions_msg.data.data[2] = static_cast<int32_t>(rear_left_wheel_position);
  wheel_positions_msg.data.data[3] = static_cast<int32_t>(rear_right_wheel_position);
  
#elif defined(CUB_TARGET_MCUB)
  int32_t right_wheel_position;
  int32_t left_wheel_position;
  if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10))) {
    while(DXL_SERIAL.available() > 0){
      DXL_SERIAL.read();
    }
    right_wheel_position = dxl.getPresentPosition(DXL1_ID);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    left_wheel_position = dxl.getPresentPosition(DXL2_ID);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    xSemaphoreGive(mutex);
  } else {
    return;
  }
  wheel_positions_msg.data.data[0] = left_wheel_position;
  wheel_positions_msg.data.data[1] = right_wheel_position;
#endif

  // メッセージのパブリッシュ
  RCCHECK(rcl_publish(&wh_pos_publisher, &wheel_positions_msg, NULL));
}

void control_motor(const geometry_msgs__msg__Twist* arg_twist) {
  double r_vel_m = arg_twist->linear.x + cub_d * arg_twist->angular.z / 1000.0; // [m/s]
  double l_vel_m = arg_twist->linear.x - cub_d * arg_twist->angular.z / 1000.0; // [m/s]
  double r_vel_r = r_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  double l_vel_r = l_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
  #ifdef CUB_TARGET_CUB2
  int r_goal_vel = (int)(r_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // right goal velocity[rpm] # TODO: check whether need to add dvidid by motor_velocity_unit
  int l_goal_vel = (int)(l_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // left goal velocity[rpm]

  vehicle_run(r_goal_vel, l_goal_vel);
  #elif defined(CUB_TARGET_MCUB)
  int32_t r_goal_vel = (int32_t)(r_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // right goal velocity
  int32_t l_goal_vel = (int32_t)(l_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // left goal velocity

  if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10))) {
    dxl.setGoalVelocity(DXL1_ID, r_goal_vel);
    delay(5);
    dxl.setGoalVelocity(DXL2_ID, l_goal_vel);
    delay(5);
    xSemaphoreGive(mutex);
  #ifdef DEBUG
    if (arg_twist->linear.x > 0) {
      leds[7] = CRGB::Blue;
      leds[17] = CRGB::Black;
    } else if (arg_twist->linear.x < 0) {
      leds[7] = CRGB::Black;
      leds[17] = CRGB::Blue;
    } else {
      leds[7] = CRGB::Black;
      leds[17] = CRGB::Black;
    }

    if (arg_twist->angular.z > 0) {
      leds[11] = CRGB::Blue;
      leds[13] = CRGB::Black;
    } else if (arg_twist->angular.z < 0) {
      leds[11] = CRGB::Black;
      leds[13] = CRGB::Blue;
    } else {
      leds[11] = CRGB::Black;
      leds[13] = CRGB::Black;
    }
    FastLED.show();
  #endif // DEBUG
  }
  #endif // CUB_TARGET
  if (uros_state == AGENT_CONNECTED) {
    wh_pos_timer_callback();
  }
}

void twist_callback(const void * msgin)
{
  if (msgin == NULL) return;
  prev_msg_time = millis();
  delay(100);
}

void motor_control_loop(void *pvParameters = nullptr) 
{
  attachInterrupt(EMERGENCY_MONITOR, set_emergency, RISING);
  while(1) {
    switch (robo_mode)
    {
    case IDLE:
  #ifdef CUB_TARGET_CUB2
      motor_stop();
  #endif
      send_twist_msg = vehicle_stop_msg();
      break;
    case REMOTE_CTRL:
      send_twist_msg = remote_twist_msg;
      break;
    case AUTONOMOUS:
      if (millis() - prev_msg_time > 3000) {
        robo_mode = IDLE;
  #ifdef CUB_TARGET_CUB2
        motor_stop();
  #endif
        subscribe_twist_msg = vehicle_stop_msg();
      }
        send_twist_msg = subscribe_twist_msg;
      break;
    case EMERGENCY:
  #ifdef CUB_TARGET_CUB2
      emergency_stop();
  #endif
      remote_twist_msg = vehicle_stop_msg();
      subscribe_twist_msg = vehicle_stop_msg();
      send_twist_msg = vehicle_stop_msg();
      break;
    default:
  #ifdef CUB_TARGET_CUB2
      motor_stop();
  #endif
      send_twist_msg = vehicle_stop_msg();
      break;
    }
    
    control_motor(&send_twist_msg);
    
    delay(30);
  }
}

void remote_control_loop(void *pvParameters){
  // initialize PS5
  connect_dualsense();
  
  while(1) {
    if (robo_mode == EMERGENCY) {
      delay(100);
      continue;
    }
  #ifdef CUB_TARGET_CUB2
    float default_linear = 1.2;
  #elif defined(CUB_TARGET_MCUB)
    float default_linear = 0.3;
  #endif
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
      } else if (ps5.L2()) {
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
      } else if (ps5.Share()){
          robo_mode = REMOTE_CTRL; //遠隔操作モード
      } else {
        remote_twist_msg = vehicle_stop_msg();
      }
    } else {
      leds[2] = CRGB::Red;
    
      if (uros_state == AGENT_CONNECTED) robo_mode = AUTONOMOUS;
      else robo_mode = IDLE;
    }
    delay(100);
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
  RCCHECK(rclc_publisher_init_best_effort(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "mros_debug_topic"));
    
#ifdef CUB_TARGET_MCUB
  // create imu_publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "pub_imu"));
#endif

  // create wh_pos_publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &wh_pos_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "wheel_positions"));


  // create subscliber
  rmw_qos_profile_t minimum_qos = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
  RCCHECK(rclc_subscription_init(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_atom", &minimum_qos));

#ifdef CUB_TARGET_MCUB
  // create imu_timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &imu_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    imu_timer_callback));
#endif
  
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &subscribe_twist_msg, &twist_callback, ON_NEW_DATA));
#ifdef CUB_TARGET_MCUB
  // RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
#endif
 
  // select semantics
  RCCHECK(rclc_executor_set_semantics(&executor, RCLCPP_EXECUTOR));

  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_publisher_fini(&debug_publisher, &node));
#ifdef CUB_TARGET_MCUB
  RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
#endif
  RCCHECK(rcl_publisher_fini(&wh_pos_publisher, &node));
  RCCHECK(rcl_subscription_fini(&subscriber, &node));
#ifdef CUB_TARGET_MCUB
  RCCHECK(rcl_timer_fini(&imu_timer));
#endif
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));
  RCCHECK(rcl_init_options_fini(&init_options));
}

void setup() {
  robo_mode = IDLE;
  pinMode(EMERGENCY_MONITOR, INPUT); 
  gpio_pulldown_dis(EMERGENCY_MONITOR);
  
#ifdef CUB_TARGET_MCUB
  auto cfg = M5.config();
  M5.begin(cfg);
#endif

  mutex = xSemaphoreCreateMutex();

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // initialize LED
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
#ifdef CUB_TARGET_CUB2
  FastLED.addLeds<NEOPIXEL, LANE_LED_DATA_PIN>(lane_led, NUM_LANE_LEDS);
  FastLED.setBrightness(200);
  fill_solid(lane_led, NUM_LANE_LEDS, CRGB( 255, 255, 255));
#elif defined(CUB_TARGET_MCUB)
  FastLED.setBrightness(20);
#endif
  leds[0] = CRGB::White;
  FastLED.show();
  delay(10);

#ifdef CUB_TARGET_CUB2
  for(int i=1;i<=4;i++){
    motor_handler.Control_Motor(0, i, Acce, Brake_P, &Receiv[i]); //4つのモーター
  }
#elif defined(CUB_TARGET_MCUB)
  initialize_dynamixel();
#endif

  // メッセージの初期化
  std_msgs__msg__Int32MultiArray__init(&wheel_positions_msg);
#ifdef CUB_TARGET_CUB2
  wheel_positions_msg.data.capacity = 4;
  wheel_positions_msg.data.size = 4;
  wheel_positions_msg.data.data = (int32_t*) malloc(4 * sizeof(int32_t));
#elif defined(CUB_TARGET_MCUB)
  wheel_positions_msg.data.capacity = 2;
  wheel_positions_msg.data.size = 2;
  wheel_positions_msg.data.data = (int32_t*) malloc(2 * sizeof(int32_t));
#endif

  xTaskCreatePinnedToCore(motor_control_loop, "motor_control_loop", 4048, NULL, 3, NULL, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(remote_control_loop, "remote_control_loop", 4048, NULL, 2, NULL, APP_CPU_NUM);
}

void loop() {
  if (digitalRead(EMERGENCY_MONITOR) == HIGH) {
#ifdef CUB_TARGET_CUB2
    emergency_stop();
    fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Red);
#endif
    robo_mode = EMERGENCY;
    leds[4] = CRGB::Red;
    FastLED.show();
  } else {
    if ((robo_mode == EMERGENCY) && (!ps5.Cross())) robo_mode = IDLE;
  }

  if (robo_mode == EMERGENCY){
    delay(100);
    return;
  }

  if (robo_mode == REMOTE_CTRL) {
    leds[4] = CRGB::Yellow;
#ifdef CUB_TARGET_CUB2
    fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Yellow);
#endif
  } else if (robo_mode == AUTONOMOUS) {
    leds[4] = CRGB::Green;
#ifdef CUB_TARGET_CUB2
    fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Green);
#endif
  } else if (robo_mode == IDLE) {
    leds[4] = CRGB::Blue;
#ifdef CUB_TARGET_CUB2
    fill_solid(lane_led, NUM_LANE_LEDS, CRGB::Blue);
#endif
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
                         uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3))
                                     ? AGENT_CONNECTED
                                     : AGENT_DISCONNECTED;);
      if (uros_state == AGENT_CONNECTED) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
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
  delay(50);
}
