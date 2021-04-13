/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "Particle.h"
#include "tracker_config.h"
#include "config_service.h"


/**
 * @brief Definition of sleep configurations.
 * Sleep can be disabled or enabled in one or more different modes.
 *
 */
enum class TrackerSleepMode {
  Disable,                        /**< Sleep is disabled */
  Enable,                         /**< Sleep is enabled */
};

// Default configurations for Sleep
constexpr TrackerSleepMode TrackerSleepDefaultMode = TrackerSleepMode::Disable;
constexpr int32_t TrackerSleepDefaultExeMinTime = 10; // seconds
constexpr int32_t TrackerSleepDefaultConnMaxTime = 90; // seconds
constexpr int32_t TrackerSleepDefaultMaxTime = 86400; // seconds
constexpr system_tick_t TrackerSleepGracefulTimeout = 5 * 1000; // milliseconds
constexpr system_tick_t TrackerSleepShutdownTimeout = 4 * 1000; // milliseconds
constexpr system_tick_t TrackerSleepResetTimeout = 5 * 1000; // milliseconds
constexpr unsigned int TrackerSleepResetTimerDelay = 5 * 1000; // milliseconds

struct tracker_sleep_config_t {
    TrackerSleepMode mode;
    int32_t execute_min_seconds;
    int32_t connecting_max_seconds;
};

/**
 * @brief Return types for time related sleep function calls.
 *
 */
enum class TrackerSleepError {
  NONE,                           /**< No errors */
  TIME_IN_PAST,                   /**< Time given is in the past */
  TIME_SKIPPED,                   /**< Time given was evaluated and not needed */
  CANCELLED,                      /**< Operation was cancelled */
};

/**
 * @brief Reason for callback context.
 *
 */
enum class TrackerSleepReason {
  PREPARE_SLEEP,                  /**< The system is preparing to sleep */
  CANCEL_SLEEP,                   /**< The system canceled sleep */
  SLEEP,                          /**< The system is going to sleep */
  WAKE,                           /**< The system woke from sleep */
  STATE_TO_CONNECTING,            /**< Sleep transition to CONNECTING */
  STATE_TO_EXECUTION,             /**< Sleep transition to EXECUTION */
  STATE_TO_SLEEP,                 /**< Sleep transition to SLEEP */
  STATE_TO_SHUTDOWN,              /**< Sleep transition to SHUTDOWN */
  STATE_TO_RESET,                 /**< Sleep transition to RESET */
};

/**
 * @brief Return structure for Tracker sleep functions.
 *
 */
struct TrackerSleepResult {
  SystemSleepResult result;       /**< System sleep result */
  TrackerSleepError error;        /**< Tracker sleep internal error result */
};

/**
 * @brief Structure to contain information related to sleep and wake.
 *
 */
struct TrackerSleepContext {
  TrackerSleepReason reason;      /**< Enumerated reason for the call */
  size_t loop;                    /**< Loop number call made */
  uint64_t lastSleepMs;           /**< The last time, in milliseconds, the system went to sleep */
  uint64_t lastWakeMs;            /**< The last time, in milliseconds, the system woke from sleep */
  uint64_t nextWakeMs;            /**< The next time, in milliseconds, the system will wake from sleep */
  uint64_t modemOnMs;             /**< The time, in milliseconds, when the modem was turned on */
};

/**
 * @brief Type definition of sleep watchdog callbacks.
 *
 */
using SleepWatchdogCallback = std::function<void(bool enable)>;

/**
 * @brief Type definition of sleep callback signature.
 *
 */
using SleepCallback = std::function<void(TrackerSleepContext context)>;

/**
 * @brief Execution states for sleep
 *
 */
enum class TrackerExecutionState {
  BOOT,
  CONNECTING,
  EXECUTION,
  SLEEP,
  SHUTDOWN,
  RESET,
};

/**
 * @brief TrackerSleep class to configure and manage sleep.
 *
 */
class TrackerSleep {
public:
  /**
   * @brief Singleton class instance access for TrackerSleep.
   *
   * @return TrackerSleep&
   */
  static TrackerSleep& instance() {
    if (!_instance) {
      _instance = new TrackerSleep();
    }
    return *_instance;
  }

  /**
   * @brief Initialize TrackerSleep.
   *
   * @retval SYSTEM_ERROR_NONE
   */
  int init(SleepWatchdogCallback watchdog = nullptr);

  /**
   * @brief Get the config mode for sleep
   *
   * @return TrackerSleepMode Enumeration for sleep configuration mode
   */
  TrackerSleepMode getConfigMode() {
    return _config_state.mode;
  }

  /**
   * @brief Indicate that sleep is disabled
   *
   * @return true Sleep is disabled
   * @return false Sleep is enabled (in some form)
   */
  bool isSleepDisabled() {
    return (_config_state.mode == TrackerSleepMode::Disable);
  }

  /**
   * @brief Get the config execution time limit
   *
   * @return int32_t Execution time limit in seconds
   */
  int32_t getConfigExecuteTime() {
    return _config_state.execute_min_seconds;
  }

  /**
   * @brief Get the config connecting limit time
   *
   * @return int32_t Connecting time limit in seconds
   */
  int32_t getConfigConnectingTime() {
    return _config_state.connecting_max_seconds;
  }

  /**
   * @brief Schedules system wake at specific time in relation to System.uptime().
   *
   * @param uptimeSeconds Absolute time, in seconds.  Another, sooner pending wake
   *                      time may take precidence.
   * @retval TrackerSleepError::NONE Time was scheduled
   * @retval TrackerSleepError::TIME_IN_PAST Given time happened in the past
   * @retval TrackerSleepError::TIME_SKIPPED Given time happens later than a sooner wake request
   */
  TrackerSleepError wakeAtSeconds(unsigned int uptimeSeconds);

  /**
   * @brief Schedules system wake at specific time in relation to millis().
   *
   * @param milliseconds Absolute time, in milliseconds.  Another, sooner pending wake
   *                     time may take precidence.
   * @retval TrackerSleepError::NONE Time was scheduled
   * @retval TrackerSleepError::TIME_IN_PAST Given time happened in the past
   * @retval TrackerSleepError::TIME_SKIPPED Given time happens later than a sooner wake request
   */
  TrackerSleepError wakeAtMilliseconds(system_tick_t milliseconds);

  /**
   * @brief Schedules system wake at specific time in relation to System.millis().
   *
   * @param milliseconds Absolute time, in milliseconds.  Another, sooner pending wake
   *                     time may take precidence.
   * @retval TrackerSleepError::NONE Time was scheduled
   * @retval TrackerSleepError::TIME_IN_PAST Given time happened in the past
   * @retval TrackerSleepError::TIME_SKIPPED Given time happens later than a sooner wake request
   */
  TrackerSleepError wakeAtMilliseconds(uint64_t milliseconds);

  /**
   * @brief Schedules system wake at specific time in relation to System.millis().
   *
   * @param milliseconds Absolute time, in std::chrono.  Another, sooner pending wake
   *                     time may take precidence.
   * @retval TrackerSleepError::NONE Time was scheduled
   * @retval TrackerSleepError::TIME_IN_PAST Given time happened in the past
   * @retval TrackerSleepError::TIME_SKIPPED Given time happens later than a sooner wake request
   */
  TrackerSleepError wakeAt(std::chrono::milliseconds ms);

  /**
   * @brief Enables system wake for a pin change.
   *
   * @param pin System pin to associate to wake.
   * @param mode System pin mode to associate to wake.
   * @retval SYSTEM_ERROR_NONE
   */
  int wakeFor(pin_t pin, InterruptMode mode);

  /**
   * @brief Enables system wake for a SystemSleepFlag change.
   *
   * @param flag SystemSleepFlag event to associate to wake.
   * @retval SYSTEM_ERROR_NONE
   */
  int wakeFor(SystemSleepFlag flag);

  /**
   * @brief Enables system wake for a System network interface change.
   *
   * @param netif Network interface event to associate to wake.
   * @retval SYSTEM_ERROR_NONE
   */
  int wakeFor(network_interface_t netif);

  /**
   * @brief Enables system wake for a System BLE change.
   *
   * @retval SYSTEM_ERROR_NONE
   */
  int wakeForBle();

  /**
   * @brief Disables system wake for a pin change.
   *
   * @param pin System pin to disassociate to wake.
   * @retval SYSTEM_ERROR_NONE
   */
  int ignore(pin_t pin);

  /**
   * @brief Disables system wake for a SystemSleepFlag change.
   *
   * @param flag SystemSleepFlag event to disassociate to wake.
   * @retval SYSTEM_ERROR_NONE
   */
  int ignore(SystemSleepFlag flag);

  /**
   * @brief Disables system wake for a System network interface change.
   *
   * @param netif Network interface event to disassociate to wake.
   * @retval SYSTEM_ERROR_NONE
   */
  int ignore(network_interface_t netif);

  /**
   * @brief Disables system wake for a System BLE change.
   *
   * @retval SYSTEM_ERROR_NONE
   */
  int ignoreBle();

  /**
   * @brief Prevent the system from going to sleep.
   *
   */
  void pauseSleep(){
    _holdSleep = true;
  }

  /**
   * @brief Allow the system to go to sleep.
   *
   */
  void resumeSleep() {
    _holdSleep = false;
  }

  /**
   * @brief Extend execution time accumulatively before going back to sleep.
   *
   * @param seconds Number of seconds to extend the execution phase.
   * @return uint32_t The new extended execution time.
   */
  uint32_t extendExecution(uint32_t seconds) {
    return _executeDurationSec += seconds;
  }

  /**
   * @brief Extend execution time from present time before going back to sleep.  If the
   * requested time period is less than the current execution time then override.
   *
   * @param seconds Number of seconds to extend the execution phase.
   * @param force Force the new extended execution time even if it occurs sooner.
   * @return uint32_t The new extended execution time.
   */
  uint32_t extendExecutionFromNow(uint32_t seconds, bool force = false) {
    auto now = System.uptime();
    auto expectedTime = _lastExecuteSec + _executeDurationSec;
    auto newTime = now + seconds;
    if (force || (newTime > expectedTime)) {
      _lastExecuteSec = now;
      _executeDurationSec = seconds;
      Log.trace("extending execution");
    }
    return _executeDurationSec;
  }

  /**
   * @brief Register a callback to be called while preparing for sleep.
   *
   * @param callback Function to call on sleep preparation
   * @retval SYSTEM_ERROR_NONE
   */
  int registerSleepPrepare(SleepCallback callback);

  /**
   * @brief Register a callback to be called immediately after cancelling sleep.
   *
   * @param callback Function to call on sleep cancellation
   * @retval SYSTEM_ERROR_NONE
   */
  int registerSleepCancel(SleepCallback callback);

  /**
   * @brief Register a callback to be called immediately prior to going to sleep.
   *
   * @param callback Function to call on sleep
   * @retval SYSTEM_ERROR_NONE
   */
  int registerSleep(SleepCallback callback);

  /**
   * @brief Register a callback to be called immediately after returning sleep.
   *
   * @param callback Function to call on sleep completion
   * @retval SYSTEM_ERROR_NONE
   */
  int registerWake(SleepCallback callback);

  /**
   * @brief Register a callback to be called immediately after sleep state change.
   *
   * @param callback Function to call on sleep state change
   * @retval SYSTEM_ERROR_NONE
   */
  int registerStateChange(SleepCallback callback);

  /**
   * @brief Indicate that the current connecting/execution phase has cellular modem and GNSS powered.
   *
   * @return true Cellular modem and GNSS are powered during this execution cycle
   * @return false  Cellular modem and GNSS are not powered during this execution cycle
   */
  bool isFullWakeCycle() {
    return (
      isSleepDisabled() ||
      _inFullWakeup
    );
  }

  /**
   * @brief Instruct the TrackerSleep class to power on the cellular modem and GNSS in short wake cycles.
   *
   * @retval SYSTEM_ERROR_NONE
   */
  int forceFullWakeCycle() {
    if (!_inFullWakeup) {
      _fullWakeupOverride = true;
    }
    return SYSTEM_ERROR_NONE;
  }

  /**
   * @brief Instruct TrackerSleep to power down the system after the current execution cycle.
   *
   */
  void forceShutdown() {
    _fullWakeupOverride = true;
    _pendingShutdown = true;
  }

  /**
   * @brief Indicate whether TrackerSleep is preparing for a pending shutdown request.
   *
   * @return true Shutdown is pending.
   * @return false Shutdown is not pending.
   */
  bool isForcedShutdownPending() {
    return _pendingShutdown;
  }

  /**
   * @brief Instruct TrackerSleep to publish a vitals message before going to sleep.
   *
   */
  void forcePublishVitals() {
    _pendingPublishVitals = true;
  }

  /**
   * @brief Main execution loop for the TrackerSleep class.  This must be executed within every system loop.
   *
   * @retval SYSTEM_ERROR_NONE
   */
  int loop();

private:
  /**
   * @brief Construct a new TrackerSleep singleton object
   *
   */
  TrackerSleep() :
    _onNetwork(false),
    _onBle(false),
    _wakeupReason(SystemSleepWakeupReason::UNKNOWN),
    _fullWakeupOverride(false),
    _inFullWakeup(true),
    _holdSleep(false),
    _pendingPublishVitals(false),
    _pendingShutdown(false),
    _pendingReset(false),
    _executionState(TrackerExecutionState::BOOT),
    _lastConnectingSec(0),
    _lastExecuteSec(0),
    _executeDurationSec(0),
    _nextWakeMs(0),
    _lastWakeMs(0),
    _lastRequestedWakeMs(0),
    _lastSleepMs(0),
    _lastModemOnMs(0),
    _lastNetworkConnectMs(0),
    _lastCloudConnectMs(0),
    _loopCount(0),
    _publishFlag(false)

    {

      _config_state = {
          .mode                     = TrackerSleepDefaultMode,
          .execute_min_seconds      = TrackerSleepDefaultExeMinTime,
          .connecting_max_seconds   = TrackerSleepDefaultConnMaxTime,
      };
    }

  /**
   * @brief Instruct TrackerSleep class that a publish occured.
   *
   */
  void annoucePublish() {
    _publishFlag = true;
    extendExecutionFromNow(_config_state.execute_min_seconds);
  }

  /**
   * @brief System callback to handle OTA events that will disable sleep.
   *
   * @param event
   * @param param
   */
  static void handleOta(system_event_t event, int param);

  /**
   * @brief Instruct TrackerSleep to reset.
   *
   * @retval SYSTEM_ERROR_NONE
   */
  int enterReset();

  /**
   * @brief Cloud callback to handle system reset
   *
   * @param status
   * @param root Passed object
   * @param context Usually this*
   * @return int Success (zero)
   */
  int handleReset(CloudServiceStatus status, JSONValue *root, const void *context);

  /**
   * @brief Schedules system wake at specific time in relation to System.millis().
   *
   * @param milliseconds Absolute time, in milliseconds.  Another, sooner pending wake
   *                     time may take precidence.
   * @retval TrackerSleepError::NONE Time was scheduled
   * @retval TrackerSleepError::TIME_IN_PAST Given time happened in the past
   * @retval TrackerSleepError::TIME_SKIPPED Given time happens later than a sooner wake request
   */
  TrackerSleepError updateNextWake(uint64_t milliseconds);

  /**
   * @brief Prepare for sleep and wakeup.
   *
   * @return TrackerSleepResult
   */
  TrackerSleepResult sleep();

  /**
   * @brief Transition to CONNECTING state
   *
   */
  void stateToConnecting();

  /**
   * @brief Transition to EXECUTE state
   *
   */
  void stateToExecute();

  /**
   * @brief Transition to SLEEP state
   *
   */
  void stateToSleep();

  /**
   * @brief Transition to SHUTDOWN state
   *
   */
  void stateToShutdown();

  /**
   * @brief Transition to RESET state
   *
   */
  void stateToReset();

  /**
   * @brief Power the cellular modem on
   *
   */
  void startModem();

  /**
   * @brief Power the cellular modem off
   *
   */
  void stopModem();

  // Singleton instance
  static TrackerSleep* _instance;

  // Callback to enable/disable watchdog
  SleepWatchdogCallback _watchdog;

  // Callback containers for sleep and wake
  Vector<SleepCallback> _onSleepPrepare;
  Vector<SleepCallback> _onSleepCancel;
  Vector<SleepCallback> _onSleep;
  Vector<SleepCallback> _onWake;
  Vector<SleepCallback> _onStateTransition;

  // Sleep conditions
  Vector<std::pair<pin_t,InterruptMode>> _onPin;
  Vector<SystemSleepFlag> _onFlag;
  bool _onNetwork;
  bool _onBle;

  // Cloud configuration for TrackerSleep
  tracker_sleep_config_t _config_state;

  // Stored wakeup reason
  SystemSleepWakeupReason _wakeupReason;

  bool _fullWakeupOverride;
  bool _inFullWakeup;
  bool _holdSleep;
  bool _pendingPublishVitals;
  bool _pendingShutdown;
  bool _pendingReset;
  TrackerExecutionState _executionState;
  uint32_t _lastConnectingSec;
  uint32_t _lastExecuteSec;
  uint32_t _executeDurationSec;
  system_tick_t _lastShutdownMs;
  system_tick_t _lastResetMs;
  uint64_t _nextWakeMs;
  uint64_t _lastWakeMs;
  uint64_t _lastRequestedWakeMs;
  uint64_t _lastSleepMs;
  uint64_t _lastModemOnMs;
  uint64_t _lastNetworkConnectMs;
  uint64_t _lastCloudConnectMs;
  size_t _loopCount;
  bool _publishFlag;
};
