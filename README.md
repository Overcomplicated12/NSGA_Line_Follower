# Zumo NSGA-II Line Follower (Loose + Recovery)

This project is an **experimental Zumo robot line follower** designed for **high-speed tuning using NSGA-II**.  
The robot intentionally allows looser line tracking to achieve faster runtimes, while using recovery behaviors to avoid runaway failures.

The workflow is:
1. Run the robot
2. Collect a single CSV summary line from Serial
3. Paste into `trials.csv`
4. Run at least 6 trials
5. Use NSGA-II to evolve better parameters
6. Paste the next suggested parameters back into the Arduino code
7. Repeat

---

## Hardware

- Pololu **Zumo Shield**
- **Arduino Uno**
- Zumo Reflectance Sensor Array
- Zumo Motors
- User Button (on Zumo)

---

## Key Features

### Loose Line Following
- Reduced twitchiness
- Turn capping
- Milder speed slowdown in corners
- Prioritizes speed over perfect centering

### Recovery Behaviors
- **Line-lost detection** (all sensors white)
- **Search pivot** to reacquire line
- **Corner-stuck escape** if error stays large too long
- Prevents “runaway” straight-line failures

### Abort During a Run
- During a run, you can type:
``` bash
FAIL
```
into the serial monitor to abort a run
- You can also hold down the zumo button for about 1s to abort however the feature does not fully work

### NSGA-II Integration
- Parameters are injected via a single macro:
  ```cpp
  #define NSGA_PARAMS kp,kd,base_speed,min_base_speed,corner1,corner2,corner3,brake_pwr
