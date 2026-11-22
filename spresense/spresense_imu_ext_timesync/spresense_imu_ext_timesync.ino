/****************************************************************************
 * spresense_imu_ext_timesync.ino
 *
 * Combines:
 * 1. cxd5602pwbimu_logger (Spresense SDK examples)
 *    - Captures 6-axis IMU (accel, gyro) + temp via CXD5602PWBIMU SPI driver
 * 2. spresense_gnss_sync (UM982 External GNSS Module Synchronization)
 *    - 1PPS external interrupt synchronization (PIN_D03)
 *    - NMEA UTC time synchronization via Serial2 (9600 bps)
 *    - UM982 Event measurement comparison on PIN_D04 (Um982EventComparator)
 *    - Provides precise UTC microsecond timestamps for IMU data
 *
 * Modified for Arduino IDE (Spresense Arduino Core)
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <Arduino.h>
#include <LowPower.h>
#include <TinyGPS++.h>
#include <Watchdog.h>
#include <stdarg.h>
#include <time.h>
#include <poll.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>

#include "Um982EventComparator.h"

/****************************************************************************
 * Hardware & Operating Configuration
 ****************************************************************************/

// Serial Communication
constexpr uint32_t SERIAL_BAUD = 230400;    // Host PC Serial (230400 bps for 240Hz binary IMU stream)
constexpr uint32_t GNSS_BAUD   = 9600;      // UM982 GNSS Serial2 (D0: RX, D1: TX)

// Pin Configuration
constexpr uint8_t PPS_PIN           = PIN_D03;  // 1PPS input from UM982 (RISING edge)
constexpr uint8_t EVENT_CAPTURE_PIN = PIN_D04;  // Shared event input (FALLING edge)
constexpr uint8_t IMU_DRDY_PIN      = PIN_D18;  // IMU DRDY interrupt input (RISING edge)

// IMU DRDY Timing Offset
// Microsecond delay offset added to DRDY edge timestamp (positive or negative)
// to calibrate internal sensor filter/pipeline delay.
constexpr int32_t DRDY_DELAY_OFFSET_US = 0;

// IMU Configuration
#define CXD5602PWBIMU_DEVPATH "/dev/imu0"
#define MAX_NFIFO             (1)

/*
 * IMU Sampling Rate (Hz): 15, 30, 60, 120, 240, 480, 960, 1920
 * Accel Range (g):        2, 4, 8, 16
 * Gyro Range (dps):       125, 250, 500, 1000, 2000, 4000
 * FIFO Threshold:         1, 2, 3, 4
 */
#define SAMPLING_RATE       240
#define ACCEL_RANGE         16
#define GYRO_RANGE          500
#define FIFO_THRESHOLD      1

// Startup and recovery
//
// Opening the host serial port asserts DTR, and DTR power-cycles this board
// through the PMIC. The host node (spresense_imu_node) reopens the port after
// 5 s without receiving any byte, so a board that stays silent is reset by the
// host. The limits below keep every wait bounded and stop printing once the
// IMU cannot be started, so that host reset still happens.
constexpr int      IMU_INIT_ATTEMPTS         = 5;     // open + configure tries per (re)initialization
constexpr uint32_t IMU_RETRY_DELAY_MS        = 200;   // pause between tries
constexpr int      IMU_FIRST_SAMPLE_TIMEOUT_MS = 200; // per-sample wait while dropping the first 50 ms
constexpr uint32_t IMU_STALL_TIMEOUT_MS      = 250;   // no sample for this long in loop() -> reinitialize
constexpr int      IMU_MAX_QUICK_RECOVERIES  = 3;     // reinitializations in a row before giving up...
constexpr uint32_t IMU_HEALTHY_STREAM_MS     = 10000; // ...unless the previous start streamed this long
constexpr uint32_t WATCHDOG_TIMEOUT_MS       = 3000;  // hardware watchdog: reboot if blocked this long
constexpr uint32_t WATCHDOG_KICK_INTERVAL_MS = 100;

// Status Bitmask Definitions
constexpr uint8_t STATUS_CLOCK_SYNCHRONIZED = 0x01; // Bit 0: Currently synchronized
constexpr uint8_t STATUS_SYNC_EVER           = 0x02; // Bit 1: Synchronized at least once since boot
constexpr uint8_t STATUS_SYNC_WITHIN_2S      = 0x04; // Bit 2: Last sync was within 2 seconds
constexpr uint8_t STATUS_SYNC_WITHIN_1M      = 0x08; // Bit 3: Last sync was within 1 minute (60s)
constexpr uint8_t STATUS_SYNC_WITHIN_1H      = 0x10; // Bit 4: Last sync was within 1 hour (3600s)
constexpr uint8_t STATUS_DRDY_TIMED          = 0x20; // Bit 5: Timestamp derived from IMU DRDY interrupt

// Binary Protocol for IMU Data (Total packet: 46 bytes fixed length)
struct __attribute__((packed)) SyncedIMUData {
  uint64_t utc_timestamp_us; // Synchronized UTC timestamp in microseconds (0 if not synchronized)
  uint32_t sensor_timestamp; // 19.2MHz clock timestamp from cxd5602pwbimu_data_t
  float temp;                // Temperature [degC]
  float gx;                  // Gyro X [rad/s]
  float gy;                  // Gyro Y [rad/s]
  float gz;                  // Gyro Z [rad/s]
  float ax;                  // Accel X [m/s^2]
  float ay;                  // Accel Y [m/s^2]
  float az;                  // Accel Z [m/s^2]
  uint8_t status;            // Status bit flags
};

const uint8_t HEADER[] = {0xAA, 0xBB, 0xCC, 0xDD};

static inline uint8_t calculate_checksum(const uint8_t* data, size_t len)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// GNSS Synchronization Parameters
constexpr uint32_t NMEA_TIMEOUT_US   = 950000;
constexpr uint32_t PRINT_INTERVAL_MS = 1000;

/****************************************************************************
 * Global State: GNSS Clock Synchronization & DRDY Interrupt
 ****************************************************************************/

TinyGPSPlus gps;

// Written by the PPS interrupt and consumed by loop().
volatile uint32_t captured_pps_us = 0;
volatile bool pps_pending = false;

// Written by the IMU DRDY interrupt on PIN_D18
volatile uint32_t captured_drdy_us = 0;
volatile bool drdy_pulse_received = false;

void onImuDrdyRise() {
  captured_drdy_us = static_cast<uint32_t>(micros());
  drdy_pulse_received = true;
}

// PPS currently waiting for its corresponding NMEA time.
uint32_t pending_pps_us = 0;
bool awaiting_nmea = false;

// Synchronized clock base: UTC at the PPS edge.
uint32_t base_unix_seconds = 0;
uint32_t base_pps_us = 0;
bool clock_synchronized = false;
bool has_synced_ever = false;
uint32_t last_sync_time_ms = 0;

uint8_t getSyncStatus(bool drdy_used = false)
{
  uint8_t status = 0;
  if (clock_synchronized) {
    status |= STATUS_CLOCK_SYNCHRONIZED;
  }
  if (has_synced_ever) {
    status |= STATUS_SYNC_EVER;
    uint32_t elapsed_ms = millis() - last_sync_time_ms;
    if (elapsed_ms <= 2000) {
      status |= STATUS_SYNC_WITHIN_2S;
    }
    if (elapsed_ms <= 60000) {
      status |= STATUS_SYNC_WITHIN_1M;
    }
    if (elapsed_ms <= 3600000) {
      status |= STATUS_SYNC_WITHIN_1H;
    }
  }
  if (drdy_used) {
    status |= STATUS_DRDY_TIMED;
  }
  return status;
}

const uint8_t DAYS_IN_MONTH[] = {
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

/****************************************************************************
 * Global State: IMU Logging
 ****************************************************************************/

static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
static int g_devfd = -1;
static uint32_t g_last_imu_sample_ms = 0;
static uint32_t g_imu_started_ms = 0;
static int g_quick_recoveries = 0;

static bool g_watchdog_armed = false;
static uint32_t g_last_watchdog_kick_ms = 0;

/****************************************************************************
 * Status Text and Watchdog
 ****************************************************************************/

// Status lines share the serial link with the binary IMU packets. They are
// plain ASCII, so they never contain the 0xAA 0xBB 0xCC 0xDD header, and they
// are written between packets. The host logs them and skips them.
static void logLine(const char* format, ...)
{
  char line[160];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  Serial.println(line);
}

static const char* bootCauseName(bootcause_e cause)
{
  switch (cause)
    {
      case POR_SUPPLY: return "power on (supply)";
      case POR_NORMAL: return "power on";
      case WDT_REBOOT: return "watchdog or self reboot";
      case WDT_RESET:  return "chip watchdog";
      default:         return "other";
    }
}

static void kickWatchdog()
{
  if (!g_watchdog_armed)
    {
      return;
    }
  Watchdog.kick();
  g_last_watchdog_kick_ms = millis();
}

// Arms the watchdog unless this boot was caused by a watchdog (or self)
// reboot. If the board hangs again right after a watchdog reboot, it stays
// silent and the host resets it through DTR, which also power-cycles the
// IMU add-on. Rebooting over and over would keep printing the boot lines,
// and the host would never time out.
static void armWatchdog(bootcause_e cause)
{
  if (cause == WDT_REBOOT || cause == WDT_RESET)
    {
      logLine("[BOOT] watchdog not armed after a watchdog reboot");
      return;
    }

  Watchdog.begin();
  Watchdog.start(WATCHDOG_TIMEOUT_MS);
  g_watchdog_armed = true;
  g_last_watchdog_kick_ms = millis();
  logLine("[BOOT] watchdog armed (%lu ms)",
          static_cast<unsigned long>(WATCHDOG_TIMEOUT_MS));
}

// Called when the IMU cannot be started. Nothing is printed after the first
// line, so the host times out and resets the board through DTR.
static void waitForHostReset(const char* reason)
{
  logLine("[IMU] %s; staying silent until the host resets the board", reason);

  if (g_devfd >= 0)
    {
      close(g_devfd);
      g_devfd = -1;
    }

  if (g_watchdog_armed)
    {
      Watchdog.stop();
      g_watchdog_armed = false;
    }

  for (;;)
    {
      delay(1000);
    }
}

/****************************************************************************
 * GNSS Synchronization Functions
 * (Preserved directly from spresense_gnss_sync.ino)
 ****************************************************************************/

bool isLeapYear(uint16_t year) {
  if (year % 400 == 0) return true;
  if (year % 100 == 0) return false;
  return year % 4 == 0;
}

uint32_t toUnixTime(uint16_t year, uint8_t month, uint8_t day,
                    uint8_t hour, uint8_t minute, uint8_t second) {
  uint32_t days = 0;

  for (uint16_t y = 1970; y < year; ++y) {
    days += isLeapYear(y) ? 366 : 365;
  }

  for (uint8_t m = 1; m < month; ++m) {
    days += DAYS_IN_MONTH[m - 1];
    if (m == 2 && isLeapYear(year)) {
      ++days;
    }
  }

  days += day - 1;
  return ((days * 24UL + hour) * 60UL + minute) * 60UL + second;
}

void onPpsRise() {
  captured_pps_us = static_cast<uint32_t>(micros());
  pps_pending = true;
}

// Non-blocking: called repeatedly from loop() until the PPS interrupt is attached.
// An unconnected PPS input is pulled HIGH by the extension board, and attaching
// a rising-edge interrupt while HIGH can stall. So the interrupt is attached
// only once the pin is observed LOW; until then IMU streaming continues.
bool pps_interrupt_attached = false;

void tryAttachPpsInterrupt() {
  if (pps_interrupt_attached) {
    return;
  }
  if (digitalRead(PPS_PIN) == HIGH) {
    return;
  }

  attachInterrupt(digitalPinToInterrupt(PPS_PIN), onPpsRise, RISING);
  pps_interrupt_attached = true;
}

void waitForImuDrdyLowAndAttachInterrupt() {
  // Wait until DRDY pin is LOW to prevent Spresense interrupt lockup when attached while HIGH.
  // At 240Hz, the period is ~4.17ms. If unconnected, INPUT_PULLDOWN is immediately LOW.
  // We use a 50ms timeout in case the pin is stuck HIGH (e.g. wiring issue).
  uint32_t start_ms = millis();
  while (digitalRead(IMU_DRDY_PIN) == HIGH) {
    if (millis() - start_ms >= 50) {
      // DRDY pin is held HIGH; do not attach interrupt to prevent stall
      return;
    }
    delay(1);
  }

  // Disable built-in debounce filter (~100us) by passing false as the 4th argument so 40us DRDY pulses are captured
  attachInterrupt(digitalPinToInterrupt(IMU_DRDY_PIN), onImuDrdyRise, RISING, false);
}

void handlePendingPps() {
  noInterrupts();
  bool pending = pps_pending;
  uint32_t pps_us = captured_pps_us;
  pps_pending = false;
  interrupts();

  if (!pending) {
    return;
  }

  pending_pps_us = pps_us;
  awaiting_nmea = true;

  // Clear a time update completed before this PPS. The next update must be
  // produced by an NMEA sentence received after this edge.
  (void)gps.time.value();
}

void readGnssInput() {
  while (Serial2.available() > 0) {
    // Check between bytes so a PPS arriving during a sentence establishes
    // the correct ordering relative to the NMEA parser.
    handlePendingPps();

    int received = Serial2.read();
    if (received >= 0) {
      char received_byte = static_cast<char>(received);
      gps.encode(received_byte);
      feedUm982EventByte(received_byte);
    }
  }

  handlePendingPps();
}

void synchronizeFromGnss() {
  if (!awaiting_nmea || !gps.time.isUpdated() ||
      !gps.time.isValid() || !gps.date.isValid()) {
    return;
  }

  uint32_t elapsed_us = static_cast<uint32_t>(micros()) - pending_pps_us;
  awaiting_nmea = false;

  if (elapsed_us >= NMEA_TIMEOUT_US) {
    return;
  }

  base_unix_seconds = toUnixTime(
    gps.date.year(), gps.date.month(), gps.date.day(),
    gps.time.hour(), gps.time.minute(), gps.time.second()
  );
  base_pps_us = pending_pps_us;
  clock_synchronized = true;
  has_synced_ever = true;
  last_sync_time_ms = millis();
}

// Reads GNSS data and updates the synchronized clock when a PPS/NMEA pair is
// complete. Call this from loop().
void updateGnssClock() {
  readGnssInput();
  synchronizeFromGnss();
}

uint64_t currentTimeUs() {
  if (!clock_synchronized) {
    return 0;
  }

  uint32_t elapsed_us = static_cast<uint32_t>(micros()) - base_pps_us;
  return static_cast<uint64_t>(base_unix_seconds) * 1000000ULL + elapsed_us;
}

bool synchronizedTimeAtMicros(uint32_t captured_micros, uint64_t* utc_us) {
  if (!clock_synchronized || utc_us == nullptr) {
    return false;
  }

  uint32_t elapsed_us = captured_micros - base_pps_us;
  *utc_us = static_cast<uint64_t>(base_unix_seconds) * 1000000ULL + elapsed_us;
  return true;
}

void formatTimestamp(uint64_t time_us, char* buffer, size_t buffer_size) {
  time_t seconds = static_cast<time_t>(time_us / 1000000ULL);
  uint16_t milliseconds = (time_us % 1000000ULL) / 1000ULL;
  struct tm* utc = gmtime(&seconds);

  if (utc == nullptr) {
    snprintf(buffer, buffer_size, "0000-00-00 00:00:00.000");
    return;
  }

  snprintf(buffer, buffer_size, "%04d-%02d-%02d %02d:%02d:%02d.%03u",
           utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
           utc->tm_hour, utc->tm_min, utc->tm_sec, milliseconds);
}

void printCurrentTime() {
  static uint32_t last_print_ms = 0;
  uint32_t now_ms = millis();

  if (now_ms - last_print_ms < PRINT_INTERVAL_MS) {
    return;
  }
  last_print_ms = now_ms;

  if (!clock_synchronized) {
    // Serial.println("[GNSS] Waiting for GNSS sync (PPS + NMEA)...");
    return;
  }

  char timestamp[32];
  formatTimestamp(currentTimeUs(), timestamp, sizeof(timestamp));
  // Serial.print("[GNSS] Precise UTC Time: ");
  // Serial.println(timestamp);
}

/****************************************************************************
 * IMU Driver & Output Functions
 ****************************************************************************/

static void logIoctlError(const char* step, int err)
{
  if (err == EBUSY)
    {
      // The driver's open() enters firmware update mode when the add-on's
      // MODE register is 0 or its two chips report different firmware
      // versions (including a chip that did not answer). In that mode open()
      // succeeds but every ioctl returns EBUSY.
      logLine("[IMU] %s failed (errno=%d): add-on is in firmware update mode",
              step, err);
    }
  else
    {
      logLine("[IMU] %s failed (errno=%d)", step, err);
    }
}

// Returns true when sensing has started.
static bool start_sensing(int fd, int rate, int adrange, int gdrange,
                          int nfifos)
{
  cxd5602pwbimu_range_t range;
  int ret;

  /* Set sampling rate (Hz) */
  ret = ioctl(fd, SNIOC_SSAMPRATE, rate);
  if (ret)
    {
      logIoctlError("set sampling rate", errno);
      return false;
    }

  /* Set dynamic ranges for accelerometer and gyroscope */
  range.accel = adrange;
  range.gyro = gdrange;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret)
    {
      logIoctlError("set dynamic range", errno);
      return false;
    }

  /* Set hardware FIFO threshold */
  ret = ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  if (ret)
    {
      logIoctlError("set FIFO threshold", errno);
      return false;
    }

  /* Start sensing */
  ret = ioctl(fd, SNIOC_ENABLE, 1);
  if (ret)
    {
      logIoctlError("enable", errno);
      return false;
    }

  return true;
}

// Reads and discards the first 50 ms of samples, which can be invalid.
// The driver's read() blocks without a timeout, so wait with poll() first.
// Returns false when a sample does not arrive in time.
static bool drop_50msdata(int fd, int samprate, int nfifo)
{
  int cnt = samprate / 20; /* data size of 50ms */

  cnt = ((cnt + nfifo - 1) / nfifo) * nfifo;
  if (cnt == 0) cnt = nfifo;

  while (cnt)
    {
      kickWatchdog();

      struct pollfd fds[1];
      fds[0].fd = fd;
      fds[0].events = POLLIN;
      fds[0].revents = 0;

      int ret = poll(fds, 1, IMU_FIRST_SAMPLE_TIMEOUT_MS);
      if (ret <= 0 || !(fds[0].revents & POLLIN))
        {
          return false;
        }

      read(fd, g_data, sizeof(g_data[0]) * nfifo);
      cnt -= nfifo;
    }

  return true;
}

// Opens /dev/imu0 and starts sensing, retrying a few times.
// The driver's open() resets the add-on (XRST, then a fixed 150 ms wait) and
// then reads the chips' firmware versions, and close() asserts XRST again.
// Closing and reopening therefore repeats the whole reset sequence.
static bool initImu()
{
  for (int attempt = 1; attempt <= IMU_INIT_ATTEMPTS; ++attempt)
    {
      kickWatchdog();
      if (attempt > 1)
        {
          delay(IMU_RETRY_DELAY_MS);
          kickWatchdog();
        }

      int fd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
      if (fd < 0)
        {
          logLine("[IMU] attempt %d/%d: open %s failed (errno=%d)",
                  attempt, IMU_INIT_ATTEMPTS, CXD5602PWBIMU_DEVPATH, errno);
          continue;
        }

      kickWatchdog();
      if (!start_sensing(fd, SAMPLING_RATE, ACCEL_RANGE, GYRO_RANGE,
                         FIFO_THRESHOLD))
        {
          logLine("[IMU] attempt %d/%d: sensor start failed",
                  attempt, IMU_INIT_ATTEMPTS);
          close(fd);
          continue;
        }

      kickWatchdog();
      if (!drop_50msdata(fd, SAMPLING_RATE, FIFO_THRESHOLD))
        {
          logLine("[IMU] attempt %d/%d: no sample within %d ms after enable",
                  attempt, IMU_INIT_ATTEMPTS, IMU_FIRST_SAMPLE_TIMEOUT_MS);
          close(fd);
          continue;
        }

      g_devfd = fd;
      g_imu_started_ms = millis();
      g_last_imu_sample_ms = g_imu_started_ms;
      logLine("[IMU] sensing started on attempt %d/%d (%d Hz)",
              attempt, IMU_INIT_ATTEMPTS, SAMPLING_RATE);
      return true;
    }

  return false;
}

// Called from loop() when no sample has arrived for IMU_STALL_TIMEOUT_MS.
// One known cause is in the driver: its worker returns without re-enabling
// the data-ready interrupt when it cannot take the buffer lock in time, and
// no further samples arrive after that.
//
// If the stream keeps stopping soon after each restart, the status lines
// alone would keep the host from timing out, so give up after a few quick
// restarts and let the host reset the board.
static void recoverStalledImu(uint32_t silent_ms)
{
  logLine("[IMU] no sample for %lu ms; reinitializing",
          static_cast<unsigned long>(silent_ms));

  if (millis() - g_imu_started_ms >= IMU_HEALTHY_STREAM_MS)
    {
      g_quick_recoveries = 0;
    }
  if (++g_quick_recoveries > IMU_MAX_QUICK_RECOVERIES)
    {
      waitForHostReset("IMU stalled repeatedly");
    }

  close(g_devfd);
  g_devfd = -1;

  if (!initImu())
    {
      waitForHostReset("reinitialization failed");
    }

  // Drop a DRDY edge captured before the restart so that it is not paired
  // with the first new sample.
  noInterrupts();
  drdy_pulse_received = false;
  interrupts();
}

static void send_imu_binary(cxd5602pwbimu_data_t *dat, int num, uint64_t sample_utc_us, uint8_t status)
{
  for (int i = 0; i < num; i++)
    {
      SyncedIMUData packet_data;
      packet_data.utc_timestamp_us = sample_utc_us;
      packet_data.sensor_timestamp = dat[i].timestamp;
      packet_data.temp = dat[i].temp;
      packet_data.gx = dat[i].gx;
      packet_data.gy = dat[i].gy;
      packet_data.gz = dat[i].gz;
      packet_data.ax = dat[i].ax;
      packet_data.ay = dat[i].ay;
      packet_data.az = dat[i].az;
      packet_data.status = status;

      uint8_t checksum = calculate_checksum((const uint8_t*)&packet_data, sizeof(packet_data));
      uint8_t packet[sizeof(HEADER) + sizeof(packet_data) + 1];
      memcpy(packet, HEADER, sizeof(HEADER));
      memcpy(packet + sizeof(HEADER), &packet_data, sizeof(packet_data));
      packet[sizeof(HEADER) + sizeof(packet_data)] = checksum;
      Serial.write(packet, sizeof(packet));
    }
}

/****************************************************************************
 * Arduino Setup and Loop
 ****************************************************************************/

void setup()
{
  int ret;

  // 1. Initialize Serial Communication
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(GNSS_BAUD);

  while (!Serial && millis() < 2000)
    {
      /* Wait up to 2 seconds for Serial Monitor connection */
    }

  bootcause_e boot_cause = LowPower.bootCause();
  logLine("[BOOT] spresense_imu_ext_timesync built %s %s, boot cause %d (%s)",
          __DATE__, __TIME__, static_cast<int>(boot_cause),
          bootCauseName(boot_cause));
  armWatchdog(boot_cause);

  // 2. Initialize CXD5602PWBIMU SPI driver (SPI bus 5)
  ret = board_cxd5602pwbimu_initialize(5);
  if (ret < 0)
    {
      logLine("[IMU] board_cxd5602pwbimu_initialize failed: %d", ret);
      waitForHostReset("driver registration failed");
    }

  // 3. Open the IMU device and start sensing (drops the first 50 ms of data)
  if (!initImu())
    {
      waitForHostReset("could not start the IMU");
    }

  // 4. Initialize GNSS PPS input (D3). The interrupt is attached later from
  // loop() once the pin is seen LOW (non-blocking, so startup is not delayed
  // while PPS is unconnected and pulled HIGH).
  pinMode(PPS_PIN, INPUT);
  tryAttachPpsInterrupt();

  // 5. Initialize UM982 Event Comparator on D4
  beginUm982EventComparison(
    Serial2, EVENT_CAPTURE_PIN, synchronizedTimeAtMicros
  );

  // 6. Initialize IMU DRDY interrupt on D27
  // Use INPUT_PULLDOWN so if D27 is unconnected/floating, it stays LOW
  // and no false DRDY interrupts occur.
  pinMode(IMU_DRDY_PIN, INPUT_PULLDOWN);
  waitForImuDrdyLowAndAttachInterrupt();

  // The first samples were consumed while dropping the first 50 ms; start the
  // stall timer from here so that the rest of setup() is not counted.
  g_last_imu_sample_ms = millis();
  logLine("[BOOT] ready, streaming IMU packets");
}

void loop()
{
  if (millis() - g_last_watchdog_kick_ms >= WATCHDOG_KICK_INTERVAL_MS)
    {
      kickWatchdog();
    }

  // 1. Attach PPS interrupt as soon as PPS is detected LOW (no-op once attached)
  tryAttachPpsInterrupt();

  // Update GNSS clock synchronization and UM982 event comparison
  updateGnssClock();
  updateUm982EventComparison();

  // 2. Discard bytes from the host. There are no serial commands: a single
  // stray byte must not be able to stop the IMU stream.
  while (Serial.available() > 0)
    {
      Serial.read();
    }

  // 3. Restart the IMU if samples have stopped
  uint32_t silent_ms = millis() - g_last_imu_sample_ms;
  if (silent_ms >= IMU_STALL_TIMEOUT_MS)
    {
      recoverStalledImu(silent_ms);
      return;
    }

  // 4. Poll IMU device with 0ms timeout (non-blocking)
  // Non-blocking poll ensures loop() continuously processes GNSS bytes on Serial2 and PPS edges
  struct pollfd fds[1];
  fds[0].fd = g_devfd;
  fds[0].events = POLLIN;

  int ret = poll(fds, 1, 0);
  if (ret > 0 && (fds[0].revents & POLLIN))
    {
      ret = read(g_devfd, g_data, sizeof(g_data[0]) * FIFO_THRESHOLD);
      if (ret == sizeof(g_data[0]) * FIFO_THRESHOLD)
        {
          g_last_imu_sample_ms = millis();

          uint32_t drdy_us = 0;
          bool has_drdy = false;

          noInterrupts();
          if (drdy_pulse_received)
            {
              drdy_us = captured_drdy_us;
              drdy_pulse_received = false;
              has_drdy = true;
            }
          interrupts();

          uint64_t sample_utc_us = 0;
          bool drdy_used = false;

          if (has_drdy)
            {
              // Apply calibration delay offset
              uint32_t effective_drdy_us = static_cast<uint32_t>(static_cast<int64_t>(drdy_us) + DRDY_DELAY_OFFSET_US);

              // Convert DRDY edge micros to synchronized UTC timestamp
              if (synchronizedTimeAtMicros(effective_drdy_us, &sample_utc_us))
                {
                  drdy_used = true;
                }
              else
                {
                  // Clock not synchronized yet; mark drdy_used true for status tracking
                  drdy_used = true;
                  sample_utc_us = 0;
                }
            }

          // Fallback: If DRDY pulse was not received (unconnected, missed, etc.),
          // use the current time when data arrived/read.
          if (!drdy_used)
            {
              sample_utc_us = currentTimeUs();
            }

          uint8_t status = getSyncStatus(drdy_used);
          send_imu_binary(g_data, FIFO_THRESHOLD, sample_utc_us, status);
        }
      else if (ret < 0)
        {
          // printf("ERROR: Read failed : %d (errno=%d)\n", ret, errno);
        }
    }
}