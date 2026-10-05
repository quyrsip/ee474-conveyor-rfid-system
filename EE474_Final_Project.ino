/**
 * @file EE474_Final_Project.ino
 * @brief Direct-register stepper motor driver for the sorting conveyor belt,
 *        with BLE and ESP-NOW command inputs routed through FreeRTOS.
 * @date 08/19/2026
 * @version 1.0
 * @author Sadriyah Farah, Valentina Aguilar-Hernandez
 *
 * @details
 * A 28BYJ-48 stepper (driven through a ULN2003 module) is stepped from a
 * hardware timer ISR using direct GPIO register writes instead of
 * digitalWrite(). All stepper state (on/off, speed, direction) is owned
 * exclusively by StepperControlTask; every other input source — the BLE
 * characteristic's write callback and the ESP-NOW receive callback —
 * only ever calls sendStepperCommand(), which pushes into
 * stepperCommandQueue. This keeps BLE and ESP-NOW, which each run in
 * their own task/callback context, from racing each other over the
 * timer or the `direction` variable.
 *
 * Input sources:
 *  - BLE: opcode byte(s) written to CHARACTERISTIC_UUID. See the protocol
 *    table above sendStepperCommand()'s BLE callback (MyCallbacks) below.
 *  - ESP-NOW: a single uint8_t from a second ESP32. Nonzero turns the
 *    conveyor off; zero turns it back on. Delivered to RemoteShutoffTask
 *    via a binary semaphore (plus a shared byte carrying the value,
 *    since a semaphore alone can't), not the command queue directly,
 *    since the ESP-NOW receive callback should do as little work as
 *    possible before returning.
 */

#include <Arduino.h>
#include "soc/gpio_struct.h"   ///< Exposes the GPIO peripheral's registers as the `GPIO` struct.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/**
 * @name BLE identifiers
 * @brief Service/characteristic UUIDs for the stepper control service.
 * @warning Generate your own UUIDs (e.g. uuidgenerator.net) rather than
 *          reusing these directly in a shipped design.
 * @{
 */
#define SERVICE_UUID        "f02f384a-971b-443b-a130-14a2056c0a9a"
#define CHARACTERISTIC_UUID "ea8c78d6-3dda-4dbf-8f0b-46b806fed36f"
/** @} */

/**
 * @name Stepper coil pin assignment
 * @brief GPIO pins driving the ULN2003's IN1-IN4 inputs.
 * @note Keep all four coil pins within GPIO 0-31 so they live in the same
 *       32-bit register bank (GPIO_OUT_REG / W1TS / W1TC). Pins 32-39 live
 *       in a second bank (GPIO_OUT1_REG / W1TS1 / W1TC1) — mixing banks
 *       would need two separate masks, so it's simplest to avoid that
 *       split entirely.
 * @{
 */
#define IN1_PIN 13
#define IN2_PIN 12
#define IN3_PIN 14
#define IN4_PIN 11
/** @} */

/**
 * @brief Precomputed output-register bitmasks for each coil pin.
 *
 * A pin's bit position in the output register IS its GPIO number (for
 * pins 0-31). Precomputing these once means the ISR only ever does an
 * OR of a few constants, no math on the hot path.
 */
static const uint32_t IN1_BIT = (1UL << IN1_PIN);
static const uint32_t IN2_BIT = (1UL << IN2_PIN);
static const uint32_t IN3_BIT = (1UL << IN3_PIN);
static const uint32_t IN4_BIT = (1UL << IN4_PIN);

/** @brief `coilBit[i]` is the output-register bitmask for coil `i` (0-3). */
static const uint32_t coilBit[4] = { IN1_BIT, IN2_BIT, IN3_BIT, IN4_BIT };

/**
 * @brief 8-step half-step sequence for a 4-wire stepper (e.g. 28BYJ-48).
 *
 * bit0 = IN1, bit1 = IN2, bit2 = IN3, bit3 = IN4.
 */
static const uint8_t stepSequence[8] = {
  0b0001, 0b0011, 0b0010, 0b0110,
  0b0100, 0b1100, 0b1000, 0b1001
};

/** @brief Current index into #stepSequence. Written only inside the timer ISR. */
volatile int32_t stepIndex = 0;

/**
 * @brief Step direction: +1 or -1.
 * @note Shared between StepperControlTask and the timer ISR — guard writes
 *       with #timerMux.
 */
volatile int8_t direction = 1;

volatile bool bleOff = true;

/** @brief Hardware timer used to generate step pulses. */
hw_timer_t *stepTimer = NULL;

/** @brief Spinlock protecting #stepIndex/#direction between task and ISR context. */
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/**
 * @name BLE command protocol
 * @brief Raw bytes written to CHARACTERISTIC_UUID and what they do.
 *
 * | Bytes                  | Meaning                                             |
 * |-------------------------|-----------------------------------------------------|
 * | `0x00`                  | Turn stepper OFF                                     |
 * | `0x01`                  | Turn stepper ON                                      |
 * | `0x02 <hi> <lo>`        | SET_SPEED; `<hi><lo>` = big-endian uint16 steps/sec  |
 *
 * Example: `{0x02, 0x01, 0xF4}` sets `0x01F4` = 500 steps/sec.
 * Any other or malformed opcode is ignored.
 * @{
 */
/** @} */

/**
 * @name ESP-NOW remote shutoff
 * @brief A second ESP32 sends a raw uint8_t: nonzero -> conveyor OFF, zero -> conveyor ON.
 *
 * The receive callback (#onDataReceived) runs in the Wi-Fi stack's own
 * task context, so it does no stepper work itself — it stores the byte
 * in #latestRemoteValue and gives #remoteDataSemaphore, which
 * #RemoteShutoffTask blocks on. A semaphore only signals "something's
 * ready," it doesn't carry a value the way a task notification does, so
 * the byte itself has to travel through a separate shared variable. The
 * actual on/off still goes through sendStepperCommand(), the same entry
 * point BLE uses, so StepperControlTask remains the only thing that ever
 * touches the timer directly.
 * @{
 */
#define ESPNOW_WIFI_CHANNEL 6

/** @brief Binary semaphore: given by #onDataReceived, taken by #RemoteShutoffTask. 
* used to communicate command from other esp32 telling us about match or no match
*/
SemaphoreHandle_t remoteDataSemaphore = NULL;

/**
 * @brief Most recent byte received over ESP-NOW.
 * @note Written by #onDataReceived, read by #RemoteShutoffTask. Safe
 *       without a mutex: a single-byte write is atomic on this
 *       architecture, and #remoteDataSemaphore already enforces a strict
 *       write-then-signal / wait-then-read handoff between the two, so
 *       there's no window where the reader could see a half-written value.
 */
volatile uint8_t latestRemoteValue = 0;
/** @} */

/**
 * @brief Command types accepted by #StepperControlTask via #stepperCommandQueue.
 */
enum StepperCommandType : uint8_t {
  CMD_TURN_ON,        ///< Start stepping at the current speed.
  CMD_TURN_OFF,       ///< Stop stepping (timer paused).
  CMD_SET_SPEED,      ///< `value` = steps/sec.
  CMD_SET_DIRECTION,  ///< `value` = +1 or -1.
};

/**
 * @brief One command posted to #stepperCommandQueue.
 */
struct StepperCommand {
  StepperCommandType type;  ///< Which action to perform.
  double value;              ///< Meaning depends on #type (speed, direction, or unused).
};

/**
 * @brief Queue of pending #StepperCommand entries.
 *
 * One task (#StepperControlTask) owns all stepper state (on/off, speed).
 * Everything else — BLE, ESP-NOW, or any future input source — only ever
 * sends a command into this queue. Nothing outside StepperControlTask
 * calls setStepRateHz() directly, so there's no race between two input
 * sources fighting over the timer at once.
 */
QueueHandle_t stepperCommandQueue = NULL;

/**
 * @brief Sole owner/consumer of stepper on/off/speed state.
 *
 * Blocks on #stepperCommandQueue and applies each command in order,
 * calling setStepRateHz() and writing #direction as needed. This is the
 * only task in the program that does either of those things.
 *
 * @param pvParameters Unused (required by the FreeRTOS task signature).
 *
 * @see sendStepperCommand()
 * @see setStepRateHz()
 * @see stepperCommandQueue
 */
void StepperControlTask(void *pvParameters) {
  bool   stepperOn    = false;
  double currentSpeed = 500.0;   // default rate once turned on

  StepperCommand cmd;
  for (;;) {
    // Block here until BLE (or anything else) sends a command — this
    // task costs nothing while idle.
    if (xQueueReceive(stepperCommandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd.type) {
        case CMD_TURN_ON:
          stepperOn = true;
          setStepRateHz(currentSpeed);
          break;

        case CMD_TURN_OFF:
          stepperOn = false;
          setStepRateHz(0);   // calls timerStop() internally
          break;

        case CMD_SET_SPEED:
          currentSpeed = cmd.value;
          if (stepperOn) setStepRateHz(currentSpeed);  // only apply live if already running
          break;

        case CMD_SET_DIRECTION:
          portENTER_CRITICAL(&timerMux);
          direction = (int8_t)cmd.value;   // shared with the ISR, so guard it
          portEXIT_CRITICAL(&timerMux);
          break;
      }
    }
  }
}

/**
 * @brief Universal entry point for requesting a stepper state change.
 *
 * Call this from a BLE characteristic's onWrite callback, the ESP-NOW
 * receive path, or any other input source instead of touching the
 * stepper directly. Safe to call from a different task context than
 * StepperControlTask — that's the point of the queue.
 *
 * @param type  Which command to send.
 * @param value Command payload; meaning depends on @p type (default 0).
 *
 * @see StepperControlTask()
 * @see MyCallbacks::onWrite()
 * @see RemoteShutoffTask()
 */
void sendStepperCommand(StepperCommandType type, double value = 0) {
  StepperCommand cmd = { type, value };
  xQueueSend(stepperCommandQueue, &cmd, pdMS_TO_TICKS(50));
}

/**
 * @brief ESP-NOW receive callback: stores the received byte and signals #RemoteShutoffTask.
 *
 * Runs in the Wi-Fi stack's own task context, not StepperControlTask's.
 * Expects exactly one payload byte; anything else is ignored. Writes the
 * byte to #latestRemoteValue, then gives #remoteDataSemaphore — the
 * semaphore only says "go look," #latestRemoteValue is what actually
 * carries "turn off" (nonzero) vs "turn on" (zero).
 *
 * @param info Sender info (unused).
 * @param data Pointer to the received payload.
 * @param len  Length of the received payload, in bytes.
 *
 * @see RemoteShutoffTask()
 * @see remoteDataSemaphore
 * @see latestRemoteValue
 */
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(uint8_t)) return;

  uint8_t value;
  memcpy(&value, data, sizeof(value));
  Serial.println("message received");

  // Write the byte first, THEN signal — RemoteShutoffTask is guaranteed
  // not to read latestRemoteValue until after this xSemaphoreGive(), so
  // the order here matters.
  latestRemoteValue = value;
  xSemaphoreGive(remoteDataSemaphore);
}

/**
 * @brief Consumes ESP-NOW shutoff/restart signals and forwards them to the stepper queue.
 *
 * Blocks on #remoteDataSemaphore until #onDataReceived() gives it, reads
 * #latestRemoteValue, then issues CMD_TURN_OFF (nonzero) or CMD_TURN_ON
 * (zero) via sendStepperCommand() — never touches the timer or
 * #direction directly.
 *
 * @param pvParameters Unused (required by the FreeRTOS task signature).
 *
 * @see onDataReceived()
 * @see sendStepperCommand()
 */
void RemoteShutoffTask(void *pvParameters) {
  for (;;) {
    // Block indefinitely until onDataReceived() gives the semaphore.
    // xSemaphoreTake() only returns pdTRUE once it's actually acquired
    // the semaphore, so — unlike a timed wait — there's no spurious
    // wakeup to guard against here.
    if (xSemaphoreTake(remoteDataSemaphore, portMAX_DELAY) == pdTRUE) {
      uint8_t value = latestRemoteValue;   // snapshot before acting on it

      if (value != 0 && !bleOff) {
        sendStepperCommand(CMD_TURN_ON);
      } else {
        sendStepperCommand(CMD_TURN_OFF);
      }
    }
  }
}

/**
 * @brief Writes one step pattern to the coil pins using direct register access.
 *
 * Bit-masks each coil's target state into one combined set-mask and one
 * combined clear-mask, then writes both in a single instruction each via
 * the GPIO peripheral's write-1-to-set/write-1-to-clear shadow registers
 * (`GPIO.out_w1ts` / `GPIO.out_w1tc`). Writing a 1 to a bit sets/clears
 * *that* GPIO and leaves every other pin's output level untouched, so no
 * read-modify-write is needed — this is ISR-safe and far cheaper than
 * digitalWrite(), which looks the pin up in a table and does
 * validation/locking on every call.
 *
 * @param pattern 4-bit coil pattern (bit0=IN1 ... bit3=IN4) from #stepSequence
 *
 * @see onStepTimer()
 * @see stepSequence
 */
void IRAM_ATTR applyStep(uint8_t pattern) {
  uint32_t setMask = 0, clearMask = 0;

  for (int coil = 0; coil < 4; coil++) {
    if (pattern & (1 << coil)) setMask   |= coilBit[coil];
    else                       clearMask |= coilBit[coil];
  }

  GPIO.out_w1ts = setMask;
  GPIO.out_w1tc = clearMask;
}

// void IRAM_ATTR buttonTurnOff() {
//     sendStepperCommand(CMD_TURN_OFF);

// }

/**
 * @brief Hardware timer ISR: advances the stepper by one half-step.
 *
 * Fires once per step interval, as configured by setStepRateHz().
 *
 * @see applyStep()
 * @see setStepRateHz()
 */
void ARDUINO_ISR_ATTR onStepTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  applyStep(stepSequence[stepIndex]);
  stepIndex = (stepIndex + direction + 8) % 8;   // +8 keeps this positive before %
  portEXIT_CRITICAL_ISR(&timerMux);
}

/**
 * @brief Sets (or stops) the stepper's step rate.
 *
 * Call this from a control task only, never from the ISR. timerAlarm() sets the alarm
 * value AND enables it in one call, and timerStop()/timerStart()
 * pause/resume the underlying counter instead.
 *
 * @param stepsPerSecond Desired step rate in steps/sec. Values <= 0 stop
 *                        the timer (equivalent to CMD_TURN_OFF).
 *
 * @see StepperControlTask()
 * @see onStepTimer()
 */
void setStepRateHz(double stepsPerSecond) {
  if (stepsPerSecond <= 0) {
    timerStop(stepTimer);   // pause the counter -> no more alarms can fire
    return;
  }
  uint64_t alarmValueUs = (uint64_t)(1000000.0 / stepsPerSecond);
  timerAlarm(stepTimer, alarmValueUs, true, 0);  // value, autoreload, reload_count (0 = unlimited)
  timerStart(stepTimer);                          // resume in case it was previously stopped
}

/**
 * @brief BLE characteristic callbacks: parses opcode writes and forwards them to the stepper queue.
 */
class MyCallbacks : public BLECharacteristicCallbacks {
  /**
   * @brief Handles a write to CHARACTERISTIC_UUID.
   *
   * Parses the opcode byte(s) per the BLE command protocol table above
   * and hands off to sendStepperCommand() immediately; does no stepper
   * work itself.
   *
   * @param pCharacteristic The characteristic that was written to.
   *
   * @see sendStepperCommand()
   */
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() < 1) return;

    uint8_t opcode = (uint8_t)rxValue[0];

    switch (opcode) {
      case 0x00:  // Turn OFF
        sendStepperCommand(CMD_TURN_OFF);
        bleOff = true;
        break;

      case 0x01:  // Turn ON
        sendStepperCommand(CMD_TURN_ON);
        bleOff = false;
        break;

      case 0x02:  // Set speed: needs 2 more bytes (big-endian uint16 steps/sec)
        if (rxValue.length() >= 3) {
          uint16_t speed = ((uint8_t)rxValue[1] << 8) | (uint8_t)rxValue[2];
          sendStepperCommand(CMD_SET_SPEED, (double)speed);
          bleOff = false;
        }
        break;

      default:
        // Unknown opcode — ignore rather than guess.
        break;
    }
  }
};

/**
 * @brief Arduino setup(): configures pins/timer, creates tasks and the
 *        command queue, then brings up ESP-NOW and BLE.
 *
 * @see StepperControlTask()
 * @see RemoteShutoffTask()
 * @see MyCallbacks
 * @see onDataReceived()
 */
void setup() {
  Serial.begin(9600);

  // Direct registers matter for the hot path (the ISR),
  // not the parts of the code that run once.
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // timerBegin() takes just a frequency and starts counting
  // immediately. 1,000,000 Hz -> 1 tick = 1 microsecond.
  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onStepTimer);
  setStepRateHz(0);   // start stopped; BLE/StepperControlTask decides when to turn on

  stepperCommandQueue = xQueueCreate(5, sizeof(StepperCommand));
  remoteDataSemaphore = xSemaphoreCreateBinary();
  xTaskCreate(StepperControlTask, "StepperControl", 4096, NULL, 1, NULL);
  xTaskCreate(RemoteShutoffTask, "RemoteShutoff", 4096, NULL, 1, NULL);

  // receives from the second ESP32. Brought up before BLE starts
  // advertising, since starting Wi-Fi/ESP-NOW while BLE is already
  // actively cycling the shared 2.4GHz radio is a known way to stall a
  // radio task long enough to trip the watchdog.
  WiFi.mode(WIFI_STA);
  
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.print("This board's MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init() failed");
  } else {
    esp_now_register_recv_cb(onDataReceived);
  }

  // Queue/task handles above must exist before BLE or ESP-NOW starts,
  // since a message could arrive the instant either connects.
  BLEDevice::init("StepperConveyor");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  // the motor starts OFF
  // and waits for a 0x01 command instead of auto-starting.
}

/**
 * @brief Arduino loop(): intentionally empty — all work happens in tasks/ISR.
 */
void loop() {
}