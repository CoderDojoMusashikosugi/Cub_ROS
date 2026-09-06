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
#include <TinyGPS++.h>
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
constexpr uint32_t SERIAL_BAUD = 1152000;   // Host PC Serial (1152000 bps recommended for high-rate IMU)
constexpr uint32_t GNSS_BAUD   = 9600;      // UM982 GNSS Serial2 (D0: RX, D1: TX)

// Pin Configuration
constexpr uint8_t PPS_PIN           = PIN_D03;  // 1PPS input from UM982 (RISING edge)
constexpr uint8_t EVENT_CAPTURE_PIN = PIN_D04;  // Shared event input (FALLING edge)

// IMU Configuration
#define CXD5602PWBIMU_DEVPATH "/dev/imu0"
#define MAX_NFIFO             (4)

/*
 * IMU Sampling Rate (Hz): 15, 30, 60, 120, 240, 480, 960, 1920
 * Accel Range (g):        2, 4, 8, 16
 * Gyro Range (dps):       125, 250, 500, 1000, 2000, 4000
 * FIFO Threshold:         1, 2, 3, 4
 */
#define SAMPLING_RATE       960
#define ACCEL_RANGE         16
#define GYRO_RANGE          500
#define FIFO_THRESHOLD      4

/*
 * Output format:
 * 0: Human-readable floating point CSV (timestamp[s], temp, gx, gy, gz, ax, ay, az)
 * 1: Official cxd5602pwbimu_logger Hex format (%08x,%08x,... - compatible with gyrocompass.py)
 * 2: GNSS-synchronized UTC microsecond CSV (utc_us, temp, gx, gy, gz, ax, ay, az)
 */
#define OUTPUT_FORMAT_FLOAT 0
#define OUTPUT_FORMAT_HEX   1
#define OUTPUT_FORMAT_UTC   2

#define CURRENT_OUTPUT_FORMAT OUTPUT_FORMAT_HEX

// GNSS Synchronization Parameters
constexpr uint32_t NMEA_TIMEOUT_US   = 950000;
constexpr uint32_t PRINT_INTERVAL_MS = 1000;

/****************************************************************************
 * Private Data Types / Helpers
 ****************************************************************************/

static inline uint32_t float_to_hex_uint32(float f)
{
  uint32_t u;
  memcpy(&u, &f, sizeof(u));
  return u;
}

/****************************************************************************
 * Global State: GNSS Clock Synchronization
 * (Preserved directly from spresense_gnss_sync.ino)
 ****************************************************************************/

TinyGPSPlus gps;

// Written by the PPS interrupt and consumed by loop().
volatile uint32_t captured_pps_us = 0;
volatile bool pps_pending = false;

// PPS currently waiting for its corresponding NMEA time.
uint32_t pending_pps_us = 0;
bool awaiting_nmea = false;

// Synchronized clock base: UTC at the PPS edge.
uint32_t base_unix_seconds = 0;
uint32_t base_pps_us = 0;
bool clock_synchronized = false;

const uint8_t DAYS_IN_MONTH[] = {
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

/****************************************************************************
 * Global State: IMU Logging
 ****************************************************************************/

static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
static int g_devfd = -1;
static bool g_logging_active = true;

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

void waitForPpsLowAndAttachInterrupt() {
  Serial.println("Waiting for PPS LOW. Connect the PPS input.");

  uint32_t last_message_ms = millis();
  while (digitalRead(PPS_PIN) == HIGH) {
    if (millis() - last_message_ms >= 1000) {
      last_message_ms = millis();
      Serial.println("Waiting for PPS LOW...");
    }
    delay(1);
  }

  attachInterrupt(digitalPinToInterrupt(PPS_PIN), onPpsRise, RISING);
  Serial.println("PPS input ready.");
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
    Serial.println("[GNSS] Waiting for GNSS sync (PPS + NMEA)...");
    return;
  }

  char timestamp[32];
  formatTimestamp(currentTimeUs(), timestamp, sizeof(timestamp));
  Serial.print("[GNSS] Precise UTC Time: ");
  Serial.println(timestamp);
}

/****************************************************************************
 * IMU Driver & Output Functions
 ****************************************************************************/

static int start_sensing(int fd, int rate, int adrange, int gdrange,
                         int nfifos)
{
  cxd5602pwbimu_range_t range;
  int ret;

  /* Set sampling rate (Hz) */
  ret = ioctl(fd, SNIOC_SSAMPRATE, rate);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return 1;
    }

  /* Set dynamic ranges for accelerometer and gyroscope */
  range.accel = adrange;
  range.gyro = gdrange;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret)
    {
      printf("ERROR: Set dynamic range failed. %d\n", errno);
      return 1;
    }

  /* Set hardware FIFO threshold */
  ret = ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  if (ret)
    {
      printf("ERROR: Set FIFO threshold failed. %d\n", errno);
      return 1;
    }

  /* Start sensing */
  ret = ioctl(fd, SNIOC_ENABLE, 1);
  if (ret)
    {
      printf("ERROR: Enable failed. %d\n", errno);
      return 1;
    }

  return 0;
}

static int drop_50msdata(int fd, int samprate, int nfifo)
{
  int cnt = samprate / 20; /* data size of 50ms */

  cnt = ((cnt + nfifo - 1) / nfifo) * nfifo;
  if (cnt == 0) cnt = nfifo;

  while (cnt)
    {
      read(fd, g_data, sizeof(g_data[0]) * nfifo);
      cnt -= nfifo;
    }

  return 0;
}

static void log2uart_hex(cxd5602pwbimu_data_t *dat, int num)
{
  for (int i = 0; i < num; i++)
    {
      printf("%08x,%08x,%08x,%08x,"
             "%08x,%08x,%08x,%08x\n",
             (unsigned int)dat[i].timestamp,
             float_to_hex_uint32(dat[i].temp),
             float_to_hex_uint32(dat[i].gx),
             float_to_hex_uint32(dat[i].gy),
             float_to_hex_uint32(dat[i].gz),
             float_to_hex_uint32(dat[i].ax),
             float_to_hex_uint32(dat[i].ay),
             float_to_hex_uint32(dat[i].az));
    }
}

static void log2uart_float(cxd5602pwbimu_data_t *dat, int num)
{
  for (int i = 0; i < num; i++)
    {
      /* timestamp is in 19.2MHz clock ticks (19,200,000 counts per second) */
      printf("%.6f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             dat[i].timestamp / 19200000.0f,
             dat[i].temp,
             dat[i].gx, dat[i].gy, dat[i].gz,
             dat[i].ax, dat[i].ay, dat[i].az);
    }
}

static void log2uart_utc(cxd5602pwbimu_data_t *dat, int num, uint64_t sample_utc_us)
{
  for (int i = 0; i < num; i++)
    {
      /* Outputs microsecond UTC timestamp + IMU values */
      printf("%llu,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             (unsigned long long)sample_utc_us,
             dat[i].temp,
             dat[i].gx, dat[i].gy, dat[i].gz,
             dat[i].ax, dat[i].ay, dat[i].az);
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

  Serial.println("\n=== Spresense IMU + GNSS TimeSync Starting ===");

  // 2. Initialize CXD5602PWBIMU SPI driver (SPI bus 5)
  ret = board_cxd5602pwbimu_initialize(5);
  if (ret < 0)
    {
      printf("ERROR: board_cxd5602pwbimu_initialize failed: %d\n", ret);
      while (1) { delay(1000); }
    }

  // 3. Open IMU character device
  g_devfd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
  if (g_devfd < 0)
    {
      printf("ERROR: Could not open %s (errno=%d)\n", CXD5602PWBIMU_DEVPATH, errno);
      while (1) { delay(1000); }
    }

  printf("Configuring IMU sensor:\n");
  printf("  Rate: %d Hz\n", SAMPLING_RATE);
  printf("  Accel Range: +-%d g\n", ACCEL_RANGE);
  printf("  Gyro Range: +-%d dps\n", GYRO_RANGE);
  printf("  FIFO Threshold: %d\n", FIFO_THRESHOLD);

  ret = start_sensing(g_devfd, SAMPLING_RATE, ACCEL_RANGE, GYRO_RANGE, FIFO_THRESHOLD);
  if (ret < 0)
    {
      printf("ERROR: Sensor start failed.\n");
      close(g_devfd);
      while (1) { delay(1000); }
    }

  // Drop first 50ms of data as it can be invalid
  drop_50msdata(g_devfd, SAMPLING_RATE, FIFO_THRESHOLD);
  printf("IMU sensing initialized.\n");

  // 4. Initialize GNSS PPS input (D3) and wait for PPS idle LOW
  // (The extension board pulls an unconnected input HIGH. Registering the
  // rising-edge interrupt while HIGH can stall, so wait for PPS idle LOW.)
  pinMode(PPS_PIN, INPUT);
  waitForPpsLowAndAttachInterrupt();

  // 5. Initialize UM982 Event Comparator on D4
  beginUm982EventComparison(
    Serial2, EVENT_CAPTURE_PIN, synchronizedTimeAtMicros
  );

  Serial.println("System Ready. Streaming IMU and monitoring GNSS Sync...\n");
}

void loop()
{
  // 1. Update GNSS clock synchronization and UM982 event comparison
  updateGnssClock();
  updateUm982EventComparison();

  // 2. Check for user commands from Serial
  if (Serial.available())
    {
      char c = Serial.read();
      if (c == 'q' || c == 's')
        {
          g_logging_active = !g_logging_active;
          printf("\n[CMD] Logging %s\n", g_logging_active ? "resumed" : "paused");
        }
      else if (c == 't')
        {
          if (clock_synchronized) {
            char ts[32];
            formatTimestamp(currentTimeUs(), ts, sizeof(ts));
            printf("\n[SYNC] Current UTC: %s (base_unix_sec=%u)\n", ts, base_unix_seconds);
          } else {
            printf("\n[SYNC] GNSS clock is not synchronized yet.\n");
          }
        }
      else if (c == 'h')
        {
          printf("\n--- Available Commands ---\n");
          printf("  's' or 'q': Pause/Resume IMU logging\n");
          printf("  't'       : Print current synchronized UTC time\n");
          printf("  'h'       : Show this help\n");
          printf("--------------------------\n");
        }
    }

  if (!g_logging_active)
    {
      delay(1);
      return;
    }

  // 3. Poll IMU device with 0ms timeout (non-blocking)
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
#if CURRENT_OUTPUT_FORMAT == OUTPUT_FORMAT_HEX
          log2uart_hex(g_data, FIFO_THRESHOLD);
#elif CURRENT_OUTPUT_FORMAT == OUTPUT_FORMAT_FLOAT
          log2uart_float(g_data, FIFO_THRESHOLD);
#elif CURRENT_OUTPUT_FORMAT == OUTPUT_FORMAT_UTC
          uint64_t sample_utc_us = currentTimeUs();
          log2uart_utc(g_data, FIFO_THRESHOLD, sample_utc_us);
#endif
        }
      else if (ret < 0)
        {
          printf("ERROR: Read failed : %d (errno=%d)\n", ret, errno);
        }
    }
}