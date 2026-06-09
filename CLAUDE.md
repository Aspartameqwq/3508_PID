# 3508_PID — STM32F407 M3508 Motor Control (4-Motor)

## Project Identity

STM32F407IGHx (168 MHz) cascaded PID servo control for **4× DJI M3508 PMSM motors** (CAN IDs 1–4) with C620 ESCs (CAN bus, 1 Mbps). Gear ratio: 3591:187. Remote control via DJI DR16 receiver (DBUS, USART3 DMA double-buffer). Debug/tuning via Ozone watch window on `volatile float` arrays.

**Control loop (per motor)**: Position PID (outer, P-only) → Speed PID (inner, PI+D with LPF) → Torque iq → CAN 0x200 (4-channel packed) → C620 ESCs

## Build & Flash

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug -G "MinGW Makefiles"
cmake --build build -j8
# Flash: Ozone → STM32F407IGHx → build/3508_PID.elf
```

Toolchain: `arm-none-eabi-gcc` (Arm GNU Toolchain). CMake ≥ 3.22. Use `-G "Unix Makefiles"` on Linux/Mac.

## File Map

```
Core/Inc/
├── main.h                ← Includes debug.h, control.h, DR16.h
├── debug.h               ← ALL debug variable externs (8 blocks) + BRAKE_HS_COUNT macro + quick-debug decls
├── control.h              ← Motor control API (per-motor motor_id param), mode macros, PID param structs
├── pid.h                  ← Generic PID controller (position-independent)
├── DR16.h                 ← DR16 receiver: packed RC_Ctl_t struct, API decls
├── chassis_kinematics.h   ← Forward + inverse kinematics API (X-type omni wheel), WHEEL_COUNT=4
├── can.h, usart.h, dma.h, gpio.h, tim.h  ← CubeMX HAL handles
├── stm32f4xx_it.h         ← ISR decls
└── stm32f4xx_hal_conf.h

Core/Src/
├── main.c                ← Entry: init, main loop dispatch (remote/speed/position), chassis brake logic
├── control.c             ← THE core: per-motor cascaded PID, CAN RX/TX (0x200 packed), startup probe, stiction, turns
├── pid.c                 ← PID algorithm: integral separation, anti-windup, D-term IIR LPF
├── debug.c               ← ALL debug variable definitions (8 blocks, defaults live here) + quick-debug functions
├── chassis_kinematics.c  ← RemoteToChassis (RC→vx/vy/ω) + ChassisToMotors (→4 speeds) + MotorsToChassis (forward)
├── DR16.c                ← DMA double-buffer + IDLE ISR + 18-byte DBUS parser
├── stm32f4xx_it.c        ← ISRs: USART3 IDLE → DR16 handler; CAN1 RX0 → HAL
└── can.c, usart.c, dma.c, gpio.c, tim.c  ← CubeMX peripheral init
```

User sources are registered in **top-level `CMakeLists.txt`** (NOT `cmake/stm32cubemx/CMakeLists.txt`).

## Key Architecture

### Per-Motor PID Cascade

Each motor (index 0–3, CAN ID 1–4) has its own independent PID controller pair:

```
target_angle_deg[i] → [Position PID i] → speed_setpoint[i] → [Speed PID i] → torque iq[i] → CAN 0x200 (packed)
                            ↑ P-only, Kd=Ki=0                     ↑ PI+D, alpha=0.15
                            ↑ output_limit=4 rad/s                 ↑ ff_gain=0
```

All 4 torque commands are packed into a single CAN 0x200 frame each control cycle (iq1–iq4 → motors 1–4).

### Control Modes (`control.h`)

Per-motor `control_mode[i]` — each motor can independently be in speed or position mode:

| Macro | Value | Semantics |
|---|---|---|
| `MOTOR_CONTROL_MODE_SPEED` | 0 | Speed loop only |
| `MOTOR_CONTROL_MODE_POSITION` | 1 | Position+Speed cascade |

| `debug_quick_mode` | Alias | Behavior |
|---|---|---|
| 0 | `DEBUG_QUICK_MODE_SPEED` | `debug_set_speed[i]` as per-motor target |
| 1 | `DEBUG_QUICK_MODE_POSITION` | `debug_set_position[i]` as per-motor target |
| 2 | `DEBUG_QUICK_MODE_REMOTE` | Unified remote mode (kinematics placeholder — TODO) |

**Default**: `DEBUG_QUICK_MODE_REMOTE` (2). The old `DEBUG_QUICK_MODE_REMOTE_POSITION` (3) has been merged into mode 2 — actual remote→motor mapping depends on future chassis kinematics.

### Debug System

All debug/tuning/diagnostic variables live in [debug.h](Core/Inc/debug.h) and [debug.c](Core/Src/debug.c) ONLY. They are organized in **8 numbered blocks** — see `## When Editing Code` for the block table. Ozone can read/write all `volatile float/uint` variables live.

`Control_ApplyPidParams()` syncs block 2/3 (PID params) to PID structs every control cycle for all 4 motors.

**Block overview** (see debug.h for full per-variable `@brief` comments):

| Block | Content | Key variables |
|-------|---------|--------------|
| 1 | Quick debug tunables | `debug_quick_mode`, brake params, `debug_remote_deadzone` |
| 2 | Speed PID (per-motor) | `debug_speed_kp[4]`, `debug_speed_ki[4]`, ... (11 arrays) |
| 3 | Position PID (per-motor) | `debug_position_kp[4]`, `debug_position_deadband_deg[4]`, ... (9 arrays) |
| 4 | Kinematics tunables | `debug_kinematics_max_speed_ms`, `debug_kinematics_bias_angle_deg`, ... |
| 5 | Runtime diagnostics (per-motor) | `debug_actual_speed_rad_s[4]`, `debug_torque_cmd[4]`, ... |
| 6 | Global diagnostics | `debug_main_loop_count`, `debug_control_task_count`, `debug_can_rx_raw_count` |
| 7 | Kinematics diagnostics | `debug_chassis_vx_ms`, `debug_chassis_vy_ms`, `debug_chassis_omega_rad_s` |
| 8 | DR16 DBUS diagnostics | `debug_dr16_raw[18]`, `debug_dr16_signal_timeout_ms`, ... |

**Turns command** (one-shot, independent of quick_mode): set `debug_run_turns` non-zero in Ozone → main loop calls `Debug_QuickRunTurns()` for all motors → auto-clears after execution. Each motor tracks its own "moving carrot" + encoder tick accumulation independently.

**Brake system** (chassis-level, two-stage, no position loop):
- Stage 0 (0~300ms): Forward kinematics → reverse chassis (vx, vy, omega) → inverse kinematics → per-motor targets
- Stage 1 (after 300ms): Speed mode + target=0 → hard stop iq=0 unconditionally
- Tune: `debug_brake_reverse_speed_ms` (linear), `debug_brake_reverse_omega_rad_s` (angular), `BRAKE_HS_COUNT` (duration)

### CAN Communication

- **TX**: One `0x200` frame/cycle packs all 4 iq values (motor ID = channel index: motor 1 → iq1, etc.)
- **RX**: `Control_OnCanRxMessage()` routes frames by StdId (0x201 → index 0, 0x204 → index 3). Motors 5–8 (0x205–0x208) are ignored.
- **Startup probe**: All 4 motors probed simultaneously — each gets `MOTOR_STARTUP_CURRENT` (2500 iq) on its channel until its first feedback frame arrives. Motors that never respond are left at 0 after timeout (1500 ms).

### Important Design Decisions

1. **Linear deadband** (not nonlinear soft-landing): `effective = |error| - deadband` — continuous transition, no speed wall at deadband edge
2. **ShortestArcError** with ±180° flip-flop prevention: when paths are near-equal (|error| ≈ 180°), locks to "decrease angle" direction to prevent D-term explosion
3. **Stiction compensation**: position-mode ONLY, linearly scaled by position error below `debug_position_stiction_threshold_deg[i]` (10° default). Full 500 iq when error > threshold, 0 at deadband.
4. **Turns speed override**: carrot's speed_setpoint bypasses position loop output_limit (4 rad/s) — directly writes `debug_turns_speed_rad_s` to speed loop input
5. **Hard stop**: speed mode with target=0 unconditionally zeros iq (no PID, no brake, no stiction). Motor coasts to stop on friction alone.
6. **Zero-cross handling**: detection zone + braking current when speed and target oppose; integral cleared when target speed jumps >2 rad/s
7. **DR16 DMA double-buffer**: CubeMX generates Normal mode → `DR16_Init()` upgrades to Circular+Double-Buffer. Must manually set `PAR = &USART3->DR` (CubeMX doesn't set it). IDLE ISR processes the buffer that CT does NOT point to.

### DR16 RC Channels

| Channel | Role | Range | Center |
|---|---|---|---|
| ch0 | Right horizontal | 364–1684 | 1024 |
| ch1 | Right vertical | 364–1684 | 1024 |
| ch2 | Left vertical | 364–1684 | 1024 |
| ch3 | Left horizontal | 364–1684 | 1024 |
| s1, s2 | Switches | 1=UP, 3=MID, 2=DOWN | — |

## When Editing Code

- **CubeMX USER CODE regions**: Only edit between `USER CODE BEGIN`/`END` markers in CubeMX-generated files (main.c, stm32f4xx_it.c, main.h). CubeMX will clobber everything else on re-generation.
- **New user modules**: Add `.c` to top-level `CMakeLists.txt` `target_sources`, keep `.h` in `Core/Inc/`.
- **Tuning defaults**: Change the initial value in `debug.c`. These are the values that take effect on boot before Ozone modifies them.
- **New debug variables**: Declare `extern volatile` in `debug.h`, define in `debug.c` (as array of `MOTOR_COUNT` if per-motor, as scalar if global), update in control loop if they're diagnostics.
- **DEBUG VARIABLES MUST LIVE IN debug.h/debug.c ONLY**: This is a hard rule. Never define or declare a `debug_xxx` or `volatile` tuning/diagnostic variable outside of `Core/Inc/debug.h` and `Core/Src/debug.c`. These two files are organized into 8 numbered blocks — always add new variables to the matching block:

  | Block | Purpose | Array/Per-Motor? |
  |-------|---------|------------------|
  | 1 | Quick debug tunable params (mode, targets, brake, turns, remote) | Scalar or [4] |
  | 2 | Speed PID params | `[MOTOR_COUNT]` per-motor |
  | 3 | Position PID params | `[MOTOR_COUNT]` per-motor |
  | 4 | Kinematics tunable params (chassis geometry, speeds, CCW, bias) | Scalar |
  | 5 | Runtime diagnostics (angle, speed, torque, error) | `[MOTOR_COUNT]` per-motor |
  | 6 | Global diagnostics (loop counts, CAN stats) | Scalar |
  | 7 | Kinematics diagnostics (vx, vy, omega, motor speeds) | Scalar + [4] |
  | 8 | DR16 DBUS diagnostics (signal quality, raw frames) | Scalar + [18] |

  When adding a new feature that needs Ozone-tunable parameters or diagnostic variables:
  1. **Identify the right block** from the table above
  2. **Add `extern volatile` declaration** in `debug.h` in that block, with `@brief` Doxygen comment
  3. **Add `volatile` definition with default** in `debug.c` in the same block, matching order
  4. **Update any other files** (main.c, control.c, chassis_kinematics.c) to USE the variable, but never to define it
  5. **Update 3508_PID.md** documentation
  6. **Update this CLAUDE.md** if the feature is architecturally significant

  **Violation examples** (things to NEVER do):
  - `volatile float my_param = 0.0f;` in chassis_kinematics.c ← WRONG
  - `extern volatile float debug_foo;` in chassis_kinematics.h ← WRONG
  - `static float brake_timer;` in main.c that you want to tune in Ozone ← WRONG (must be `volatile` in debug.c)

## Project Documentation

[3508_PID.md](3508_PID.md) — comprehensive project datasheet (v1.2). Includes full debug variable tables, PID tuning guide, mode switching guide, DBUS/CAN protocol reference, CubeMX config, troubleshooting. Update it when adding new variables or features.

## Git

Initialized, remote at `https://github.com/Aspartameqwq/3508_PID.git`. Proxy: `127.0.0.1:7897`.
