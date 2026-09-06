/****************************************************************************
 * spresense_imu_ext_timesync.ino
 *
 * Based on examples/cxd5602pwbimu/cxd5602pwbimu_logger_main.c
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
 *
 * Modified for Arduino IDE (Spresense Arduino Core)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <Arduino.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <poll.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH      "/dev/imu0"
#define MAX_NFIFO (4)

/*
 * Configuration Parameters
 *
 * Sampling Rate (Hz): 15, 30, 60, 120, 240, 480, 960, 1920
 * Accel Range (g):    2, 4, 8, 16
 * Gyro Range (dps):   125, 250, 500, 1000, 2000, 4000
 * FIFO Threshold:     1, 2, 3, 4
 */
#define SAMPLING_RATE       960
#define ACCEL_RANGE         16
#define GYRO_RANGE          500
#define FIFO_THRESHOLD      4

/* Baud rate for Serial communication (1152000 recommended for high rate logging) */
#define SERIAL_BAUDRATE     1152000

/*
 * Output format:
 * 1: cxd5602pwbimu_logger official hex format (%08x,%08x,... - compatible with gyrocompass.py)
 * 0: Human-readable floating point CSV format (timestamp[s], temp, gx, gy, gz, ax, ay, az)
 */
#define OUTPUT_FORMAT_HEX   1

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
 * Private Data
 ****************************************************************************/

static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
static int g_devfd = -1;
static bool g_logging_active = true;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int start_sensing(int fd, int rate, int adrange, int gdrange,
                         int nfifos)
{
  cxd5602pwbimu_range_t range;
  int ret;

  /*
   * Set sampling rate. Available values (Hz) are below.
   * 15 (default), 30, 60, 120, 240, 480, 960, 1920
   */
  ret = ioctl(fd, SNIOC_SSAMPRATE, rate);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return 1;
    }

  /*
   * Set dynamic ranges for accelerometer and gyroscope.
   * Available values are below.
   * accel: 2 (default), 4, 8, 16
   * gyro: 125 (default), 250, 500, 1000, 2000, 4000
   */
  range.accel = adrange;
  range.gyro = gdrange;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret)
    {
      printf("ERROR: Set dynamic range failed. %d\n", errno);
      return 1;
    }

  /*
   * Set hardware FIFO threshold.
   * Increasing this value will reduce the frequency with which data is
   * received.
   */
  ret = ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  if (ret)
    {
      printf("ERROR: Set FIFO threshold failed. %d\n", errno);
      return 1;
    }

  /*
   * Start sensing, user cannot change any configurations after this.
   */
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
  int i;
  for (i = 0; i < num; i++)
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
  int i;
  for (i = 0; i < num; i++)
    {
      /* timestamp is in 19.2MHz clock ticks (19,200,000 counts per second) */
      printf("%.6f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             dat[i].timestamp / 19200000.0f,
             dat[i].temp,
             dat[i].gx, dat[i].gy, dat[i].gz,
             dat[i].ax, dat[i].ay, dat[i].az);
    }
}

/****************************************************************************
 * Public Functions (Arduino standard API)
 ****************************************************************************/

void setup()
{
  int ret;

  /* Initialize Serial port */
  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial && millis() < 2000)
    {
      /* Wait up to 2 seconds for Serial Monitor connection */
    }

  printf("\n=== CXD5602PWBIMU Logger (Arduino IDE) ===\n");

  /*
   * Initialize CXD5602PWBIMU driver on SPI bus 5.
   * This creates the /dev/imu0 device node.
   */
  ret = board_cxd5602pwbimu_initialize(5);
  if (ret < 0)
    {
      printf("ERROR: board_cxd5602pwbimu_initialize failed: %d\n", ret);
      while (1) { delay(1000); }
    }

  /* Open IMU character device */
  g_devfd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
  if (g_devfd < 0)
    {
      printf("ERROR: Could not open %s (errno=%d)\n", CXD5602PWBIMU_DEVPATH, errno);
      while (1) { delay(1000); }
    }

  printf("Configuring sensor:\n");
  printf("  Rate: %d Hz\n", SAMPLING_RATE);
  printf("  Accel Range: +-%d g\n", ACCEL_RANGE);
  printf("  Gyro Range: +-%d dps\n", GYRO_RANGE);
  printf("  FIFO Threshold: %d\n", FIFO_THRESHOLD);
  printf("  Output Format: %s\n", OUTPUT_FORMAT_HEX ? "Hex (Official Logger Format)" : "Float CSV");

  /* Setup Sensor and start sensing */
  ret = start_sensing(g_devfd, SAMPLING_RATE, ACCEL_RANGE, GYRO_RANGE, FIFO_THRESHOLD);
  if (ret < 0)
    {
      printf("ERROR: Sensor start failed.\n");
      close(g_devfd);
      while (1) { delay(1000); }
    }

  /* Drop first 50ms of data, because it is invalid */
  drop_50msdata(g_devfd, SAMPLING_RATE, FIFO_THRESHOLD);

  printf("IMU sensing started.\n\n");
}

void loop()
{
  struct pollfd fds[1];
  int ret;

  /* Check for user commands from Serial */
  if (Serial.available())
    {
      char c = Serial.read();
      if (c == 'q' || c == 's')
        {
          g_logging_active = !g_logging_active;
          printf("\nLogging %s\n", g_logging_active ? "resumed" : "paused");
        }
      else if (c == 'h')
        {
          printf("\n--- Commands ---\n");
          printf("  's' or 'q': Pause/Resume logging\n");
          printf("  'h': Show this help\n");
          printf("----------------\n");
        }
    }

  if (!g_logging_active)
    {
      delay(10);
      return;
    }

  fds[0].fd = g_devfd;
  fds[0].events = POLLIN;

  /* Wait for IMU FIFO data */
  ret = poll(fds, 1, 100);
  if (ret > 0 && (fds[0].revents & POLLIN))
    {
      ret = read(g_devfd, g_data, sizeof(g_data[0]) * FIFO_THRESHOLD);
      if (ret == sizeof(g_data[0]) * FIFO_THRESHOLD)
        {
#if OUTPUT_FORMAT_HEX
          log2uart_hex(g_data, FIFO_THRESHOLD);
#else
          log2uart_float(g_data, FIFO_THRESHOLD);
#endif
        }
      else if (ret < 0)
        {
          printf("ERROR: Read failed : %d (errno=%d)\n", ret, errno);
        }
    }
  else if (ret < 0)
    {
      printf("ERROR: Poll error (errno=%d)\n", errno);
    }
}