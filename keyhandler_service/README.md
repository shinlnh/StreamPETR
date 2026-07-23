# Keyhandler Service for Simulating EPS

## Features

- Captures and processes key events (press, hold, release).
- Supports auto-repeat functionality for keys held down.
- Adjusts vehicle control variables:
  - Throttle slider
  - Brake slider
  - Steering slider
- Sends CAN frames to:
  - Update driving modes (Idle, LKS, ACC, AEB)
  - Adjust ACC following distance and speed limits
  - Control gear status via the Gateway
  - Enable ADAS capture requests

---

## Key Bindings and Effects

| Key       | Action                                                                                 |
|-----------|----------------------------------------------------------------------------------------|
| `W`       | Increase throttle slider (while pressed). Reset throttle when released.               |
| `S`       | Increase brake slider and send Idle driving mode when pressed. Reset brake when released. |
| `A`       | Decrease steering slider to the left. Reset steering when released.                   |
| `D`       | Increase steering slider to the right. Reset steering when released.                  |
| `Q`       | On release, send gear request with value 5 to Gateway (special gear request).         |
| `0`       | On release, send Idle driving mode command.                                           |
| `1`       | On release, send LKS (Lane Keeping System) driving mode command.                      |
| `2`       | On release, send ACC (Adaptive Cruise Control) driving mode command.                  |
| `6`       | On release, send AEB (Automatic Emergency Braking) enable command.                    |
| `UP`      | On press, increase ACC following distance.                                           |
| `DOWN`    | On press, decrease ACC following distance.                                           |
| `LEFT`    | On press, decrease ACC speed limit.                                                  |
| `RIGHT`   | On press, increase ACC speed limit.                                                  |
| `ENTER`   | On release, send special gear request (value 6) to enable capture for ADAS service.  |

---

## Code Structure

- **KeyWorker class**  
  Handles key event processing in two threads:
  - `processThread`: Consumes key events and updates state.
  - `repeatThread`: Handles auto-repeat behavior while a key is held.

- **Key event handling:**  
  Events are pushed into a thread-safe queue and processed sequentially to update internal state and vehicle control variables.

- **CAN communication:**  
  Uses a `CanManager` instance to send packed CAN frames reflecting the current control state or commands.

- **Throttle, brake, steering variables:**  
  Externally defined volatile floats (`m_thottleSlider`, `m_brakeSlider`, `m_steerSlider`) representing vehicle control signals, modified based on key events.

---

## Usage

1. **Instantiate** a `KeyWorker` object for each key you want to monitor, providing the key code and a pointer to the `CanManager` instance.

2. **Push events** into the worker using `pushEvent(const KeyEvent&)` as key events occur.

3. The class manages threads internally to process events and send appropriate CAN frames.

4. **Clean up** by destructing the `KeyWorker` instance to properly stop threads.

---

## Dependencies

- Linux CAN headers: `<linux/can.h>`, `<linux/can/raw.h>`
- External headers: `common.h`, `CanUtils.h`, `tesla.h`, `KeyWorker.h`
- `CanManager` class for CAN communication (not included here)
- Threading and synchronization via `<thread>`, `<mutex>`, `<condition_variable>`

---

## Notes

- The auto-repeat interval is set to 100 ms.
- Steering input is clamped between -1.0 and 1.0.
- Throttle and brake sliders are clamped between 0.0 and 1.0.
- Gear requests use values 5 and 6 for special Gateway commands.
- The module assumes external management of the CAN interface and key event sources.

---