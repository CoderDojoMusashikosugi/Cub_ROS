#include "Um982EventComparator.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

namespace {

// GPS time was 18 seconds ahead of UTC at the time this program was written.
// Update this value if a future leap second changes GPS-UTC.
constexpr uint8_t GPS_UTC_LEAP_SECONDS = 18;
constexpr uint64_t GPS_EPOCH_UNIX_SECONDS = 315964800ULL;
constexpr uint32_t BUTTON_GUARD_US = 100000;
constexpr uint32_t PAIR_TIMEOUT_US = 2000000;
constexpr size_t EVENT_LINE_SIZE = 192;

Stream* um982_serial = nullptr;
UtcAtMicrosFunction utc_converter = nullptr;

volatile uint32_t interrupt_capture_us = 0;
volatile bool interrupt_pending = false;
volatile uint32_t previous_interrupt_us = 0;

bool gpio_event_pending = false;
uint32_t gpio_event_us = 0;

struct Um982Event {
  bool pending;
  uint8_t event_id;
  uint8_t status;
  uint32_t received_us;
  uint64_t utc_us;
};

Um982Event um982_event = {};

char event_line[EVENT_LINE_SIZE];
size_t event_line_length = 0;
bool receiving_unicore_line = false;

uint32_t measurement_count = 0;
int64_t minimum_delta_us = INT64_MAX;
int64_t maximum_delta_us = INT64_MIN;
double mean_delta_us = 0.0;

void onEventFall() {
  uint32_t now_us = static_cast<uint32_t>(micros());
  if (static_cast<uint32_t>(now_us - previous_interrupt_us) < BUTTON_GUARD_US) {
    return;
  }

  previous_interrupt_us = now_us;
  interrupt_capture_us = now_us;
  interrupt_pending = true;
}

bool parseUnsigned(const char* text, uint32_t* value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char* end = nullptr;
  unsigned long parsed = strtoul(text, &end, 10);
  if (*end != '\0') {
    return false;
  }

  *value = static_cast<uint32_t>(parsed);
  return true;
}

void parseEventFlag(char* line) {
  constexpr char PREFIX[] = "#EVENTFLAGA,";
  if (strncmp(line, PREFIX, sizeof(PREFIX) - 1) != 0) {
    return;
  }

  char* body = strchr(line, ';');
  char* checksum = strchr(line, '*');
  if (body == nullptr || checksum == nullptr || body >= checksum) {
    return;
  }

  *checksum = '\0';
  ++body;

  // EVENTFLAGA body:
  // eventID,status,reserved,reserved,week,second,nanosecond,...
  const char* fields[7] = {};
  char* save = nullptr;
  char* field = strtok_r(body, ",", &save);
  for (size_t index = 0; index < 7 && field != nullptr; ++index) {
    fields[index] = field;
    field = strtok_r(nullptr, ",", &save);
  }

  uint32_t event_id = 0;
  uint32_t status = 0;
  uint32_t week = 0;
  uint32_t second = 0;
  uint32_t nanosecond = 0;
  if (!parseUnsigned(fields[0], &event_id) ||
      !parseUnsigned(fields[1], &status) ||
      !parseUnsigned(fields[4], &week) ||
      !parseUnsigned(fields[5], &second) ||
      !parseUnsigned(fields[6], &nanosecond) ||
      second >= 604800UL || nanosecond >= 1000000000UL) {
    Serial.println("[EVENT] Invalid UM982 EVENTFLAGA message.");
    return;
  }

  uint64_t gps_seconds = GPS_EPOCH_UNIX_SECONDS +
                         static_cast<uint64_t>(week) * 604800ULL + second;
  um982_event.utc_us =
    (gps_seconds - GPS_UTC_LEAP_SECONDS) * 1000000ULL + nanosecond / 1000UL;
  um982_event.event_id = static_cast<uint8_t>(event_id);
  um982_event.status = static_cast<uint8_t>(status);
  um982_event.received_us = static_cast<uint32_t>(micros());
  um982_event.pending = true;
}

void formatUtc(uint64_t utc_us, char* buffer, size_t buffer_size) {
  time_t seconds = static_cast<time_t>(utc_us / 1000000ULL);
  uint32_t microseconds = static_cast<uint32_t>(utc_us % 1000000ULL);
  struct tm* utc = gmtime(&seconds);

  if (utc == nullptr) {
    snprintf(buffer, buffer_size, "0000-00-00 00:00:00.000000");
    return;
  }

  snprintf(buffer, buffer_size,
           "%04d-%02d-%02d %02d:%02d:%02d.%06lu",
           utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
           utc->tm_hour, utc->tm_min, utc->tm_sec,
           static_cast<unsigned long>(microseconds));
}

void printComparison() {
  uint64_t spresense_utc_us = 0;
  if (utc_converter == nullptr ||
      !utc_converter(gpio_event_us, &spresense_utc_us)) {
    Serial.println(
      "[EVENT] Spresense clock is not synchronized; measurement discarded.");
    return;
  }

  int64_t delta_us = static_cast<int64_t>(spresense_utc_us) -
                     static_cast<int64_t>(um982_event.utc_us);
  ++measurement_count;
  if (delta_us < minimum_delta_us) minimum_delta_us = delta_us;
  if (delta_us > maximum_delta_us) maximum_delta_us = delta_us;
  mean_delta_us +=
    (static_cast<double>(delta_us) - mean_delta_us) / measurement_count;

  char spresense_time[32];
  char um982_time[32];
  char delta_text[32];
  char minimum_text[32];
  char maximum_text[32];
  formatUtc(spresense_utc_us, spresense_time, sizeof(spresense_time));
  formatUtc(um982_event.utc_us, um982_time, sizeof(um982_time));
  snprintf(delta_text, sizeof(delta_text), "%+lld",
           static_cast<long long>(delta_us));
  snprintf(minimum_text, sizeof(minimum_text), "%lld",
           static_cast<long long>(minimum_delta_us));
  snprintf(maximum_text, sizeof(maximum_text), "%lld",
           static_cast<long long>(maximum_delta_us));

  Serial.println("[EVENT] ------------------------------");
  Serial.print("[EVENT] Spresense UTC: ");
  Serial.println(spresense_time);
  Serial.print("[EVENT] UM982 UTC:     ");
  Serial.println(um982_time);
  Serial.print("[EVENT] Delta (Spresense - UM982): ");
  Serial.print(delta_text);
  Serial.println(" us");
  Serial.print("[EVENT] Samples: ");
  Serial.print(measurement_count);
  Serial.print(", mean: ");
  Serial.print(mean_delta_us, 1);
  Serial.print(" us, min: ");
  Serial.print(minimum_text);
  Serial.print(" us, max: ");
  Serial.print(maximum_text);
  Serial.println(" us");

  // Build 9669 and later report time validity in status bits 0 and 3,
  // and PPS validity in bit 1.
  if ((um982_event.status & 0x0B) != 0x0B) {
    Serial.print("[EVENT] Warning: UM982 time/PPS status is not fully valid (status=");
    Serial.print(um982_event.status);
    Serial.println(").");
  }
}

}  // namespace

void beginUm982EventComparison(Stream& gnss_serial,
                               uint8_t event_pin,
                               UtcAtMicrosFunction utc_at_micros) {
  um982_serial = &gnss_serial;
  utc_converter = utc_at_micros;

  pinMode(event_pin, INPUT_PULLUP);
  // Serial.println("Release the EVENT button to make D4 HIGH.");
  while (digitalRead(event_pin) == LOW) {
    delay(1);
  }

  attachInterrupt(digitalPinToInterrupt(event_pin), onEventFall, FALLING);

  // EVENTFLAGA supplies the receiver timestamp for each EVENT edge. GGA is
  // required by the UM982 specification. These settings are not saved to
  // flash, so the receiver's permanent configuration is left unchanged.
  um982_serial->println("CONFIG EVENT ENABLE NEGATIVE 100");
  um982_serial->println("GPGGA 1");
  um982_serial->println("EVENTFLAGA ONCHANGED");

  // Serial.println("UM982 EVENT comparison ready on D4 (active LOW).");
}

void feedUm982EventByte(char received) {
  if (received == '#') {
    receiving_unicore_line = true;
    event_line_length = 0;
  }

  if (!receiving_unicore_line) {
    return;
  }

  if (received == '\r') {
    return;
  }

  if (received == '\n') {
    event_line[event_line_length] = '\0';
    parseEventFlag(event_line);
    receiving_unicore_line = false;
    event_line_length = 0;
    return;
  }

  if (event_line_length + 1 < EVENT_LINE_SIZE) {
    event_line[event_line_length++] = received;
  } else {
    receiving_unicore_line = false;
    event_line_length = 0;
  }
}

void updateUm982EventComparison() {
  noInterrupts();
  bool captured = interrupt_pending;
  uint32_t captured_us = interrupt_capture_us;
  interrupt_pending = false;
  interrupts();

  if (captured) {
    gpio_event_us = captured_us;
    gpio_event_pending = true;
  }

  if (gpio_event_pending && um982_event.pending) {
    printComparison();
    gpio_event_pending = false;
    um982_event.pending = false;
    return;
  }

  uint32_t now_us = static_cast<uint32_t>(micros());
  if (gpio_event_pending &&
      static_cast<uint32_t>(now_us - gpio_event_us) > PAIR_TIMEOUT_US) {
    Serial.println("[EVENT] No UM982 EVENTFLAGA received within 2 seconds.");
    gpio_event_pending = false;
  }

  if (um982_event.pending &&
      static_cast<uint32_t>(now_us - um982_event.received_us) > PAIR_TIMEOUT_US) {
    Serial.println("[EVENT] UM982 event received without a D4 falling edge.");
    um982_event.pending = false;
  }
}
