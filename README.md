# Bowling

Real Physics Bowling Simulation. It simulates ball movement on a complex bowling pattern based on actual physics.

Feel free to contribute to this code. It is already fairly thorough, but still needs a few tweaks.

## Setup

```text
fileholder/
└── lib/
    ├── main.dart
    ├── models/
    │   └── physics.dart
    └── painters/
        └── lane_painter.dart

cd fileholder
flutter create .

# Startup

cd fileholder
flutter run

# Alternatively:

flutter run --windows
flutter run --linux
flutter run --ios
flutter run --android
flutter run --macos
flutter run --web

# Known errors

No real-life tests yet. If you have input, please contribute here, especially if you have experience with specific bowling patterns.

Model tweaks are as follows.

## Lane Breakdown Settings

| Mode           | OIL_PICKUP_RATE | OIL_DEPOSIT_RATE | OIL_CARRY_BLEND | Effect                                                     |
| -------------- | --------------- | ---------------- | --------------- | ---------------------------------------------------------- |
| **Slow**       | 0.020           | 0.012            | 0.25            | Minimal transition, lane stays stable longer               |
| **Medium**     |  0.36           | 0.20             | 1.7             | Balanced, realistic breakdown after full round             |
| **Fast**       | 0.72            | 0.4              | 3.4             | Faster burn, noticeable carrydown                          |
| **Aggressive** | 2.88            | 1.6              | 13.6            | Rapid transition, strong carrydown, backend gets sensitive |

---

### Quick Guide

* Increase **OIL_PICKUP_RATE** → fronts burn faster
* Increase **OIL_DEPOSIT_RATE** → more oil moves downlane
* Increase **OIL_CARRY_BLEND** → carrydown has stronger impact


# tuning parameters

## Bowling sim tuning guide

This is the clean priority order for changing **timing** and **strength** of **hook** and **roll**.

The rule is simple:

* **priority1** = first knob to try
* **priority2** = second knob if priority1 is not enough
* **priority3** = shaping knob
* **priority4** = last-resort trigger adjustment

Do **not** start with thresholds unless the earlier priorities are already close.

---

## Earlier hook

```dart
const double STRIBECK_A = 0.0024;
// priority1 → change this first
// gives friction something to build from
// without enough base friction, the ball cannot read the lane early
// use this first when hook is too late overall

const double STRIBECK_B = 0.010;
// priority2 → change this second
// moves the transition earlier
// this is the main hook timing control
// lower B = friction ramps up sooner out of oil

final double tractionY =
    (1.35 /*priority3 → change this third
             makes hook readiness build sooner
             use this after A and B are close
             higher = earlier recognition of friction zones */ *
            (1.0 - hLocal.clamp(0.0, 1.0)) *
            (1.0 - 0.75 * slipRatio.clamp(0.0, 1.0)))
        .clamp(0.0, 1.0);

if (tractionY > 0.12) return 'hook';
// priority4 → change this last
// lowers the threshold required to enter hook phase
// this is only the trigger, not the cause
// use this only after the physics above are already close
```

---

## Later hook

```dart
const double STRIBECK_A = 0.0015;
// priority1 → change this first
// lowers the friction available to start reading the lane
// use this first when hook is happening too early overall

const double STRIBECK_B = 0.020;
// priority2 → change this second
// moves the transition later
// this is the main hook timing delay control
// higher B = friction ramps up later and smoother

final double tractionY =
    (1.00 /*priority3 → change this third
             slows hook readiness buildup
             lower = later recognition of friction zones */ *
            (1.0 - hLocal.clamp(0.0, 1.0)) *
            (1.0 - slipRatio.clamp(0.0, 1.0)))
        .clamp(0.0, 1.0);

if (tractionY > 0.18) return 'hook';
// priority4 → change this last
// raises the threshold required to enter hook phase
// only use this after the earlier timing knobs are close
```

---

## Stronger hook

```dart
tyScale = 1.10;
// priority1 → change this first
// directly increases sideways force during hook phase
// this is the main hook strength knob

const double STRIBECK_A = 0.0026;
// priority2 → change this second
// raises friction available in transition and backend
// supports stronger hook, but affects more than just hook phase

final double muBase =
    muRaw.clamp(MU_ROLL_RES, muDryEff * 0.85);
// priority3 → change this third
// raises the total friction ceiling
// gives more total motion overall, including hook and roll
```

---

## Weaker hook

```dart
tyScale = 0.70;
// priority1 → change this first
// directly reduces sideways force during hook phase
// this is the cleanest way to soften hook

const double STRIBECK_A = 0.0015;
// priority2 → change this second
// lowers friction available in transition and backend

final double muBase =
    muRaw.clamp(MU_ROLL_RES, muDryEff * 0.70);
// priority3 → change this third
// lowers the total friction ceiling
// reduces overall motion
```

---

## Earlier roll

```dart
if (slipRatio < 0.08 /*priority1 → change this first
                        allows roll to happen sooner
                        higher slip tolerance = earlier roll entry */ &&
    tractionX > 0.42 /*priority2 → change this second
                        makes roll easier to qualify for
                        lower requirement = earlier roll */)
  return 'roll';

final double tractionX =
    (1.10 /*priority3 → change this third
             builds forward grip faster
             helps the ball transition from hook to roll sooner */ *
            (1.0 - (slipX.abs() / vx.clamp(0.3, 30.0))))
        .clamp(0.0, 1.0);
```

---

## Later roll

```dart
if (slipRatio < 0.05 /*priority1 → change this first
                        makes roll require less slip
                        lower slip tolerance = later roll entry */ &&
    tractionX > 0.55 /*priority2 → change this second
                        makes roll harder to qualify for
                        higher requirement = later roll */)
  return 'roll';

final double tractionX =
    (0.95 /*priority3 → change this third
             slows forward grip buildup
             keeps the ball in hook longer before roll */ *
            (1.0 - (slipX.abs() / vx.clamp(0.3, 30.0))))
        .clamp(0.0, 1.0);
```

---

## Stronger roll

```dart
txScale = 1.05;
// priority1 → change this first
// increases forward drive during roll phase
// makes the ball continue through the pins harder

tyScale = 0.15;
// priority2 → change this second
// reduces leftover sideways motion in roll
// helps the ball straighten out and drive more cleanly
```

---

## Weaker roll

```dart
txScale = 0.85;
// priority1 → change this first
// reduces forward drive during roll phase

tyScale = 0.35;
// priority2 → change this second
// allows more leftover sideways motion in roll
// makes roll less defined and more curvy
```

---

## What each primary knob is for

### Hook timing

Use `STRIBECK_B` first.

* lower `STRIBECK_B` = earlier hook
* higher `STRIBECK_B` = later hook

### Hook strength

Use hook `tyScale` first.

* higher `tyScale` = stronger hook
* lower `tyScale` = weaker hook

### Roll timing

Use the roll condition first.

* higher slip threshold = earlier roll
* lower tractionX threshold = earlier roll
* lower slip threshold = later roll
* higher tractionX threshold = later roll

### Roll strength

Use `txScale` first, then `tyScale`.

* higher `txScale` = stronger forward roll
* lower `tyScale` = cleaner, stronger roll shape

---

## Priority rule

```dart
// PRIORITY RULE
// priority1 = first knob to try
// priority2 = second knob if priority1 is not enough
// priority3 = shaping knob
// priority4 = last-resort trigger adjustment
//
// Never start with thresholds unless the earlier physics are already close.
```

---

## Quick lookup table

| Goal          | First change        | Second change            | Third change      | Last change          |
| ------------- | ------------------- | ------------------------ | ----------------- | -------------------- |
| Earlier hook  | `STRIBECK_A` up     | `STRIBECK_B` down        | `tractionY` up    | lower hook threshold |
| Later hook    | `STRIBECK_A` down   | `STRIBECK_B` up          | `tractionY` down  | raise hook threshold |
| Stronger hook | hook `tyScale` up   | `STRIBECK_A` up          | `muBase` cap up   | —                    |
| Weaker hook   | hook `tyScale` down | `STRIBECK_A` down        | `muBase` cap down | —                    |
| Earlier roll  | slip threshold up   | tractionX threshold down | `tractionX` up    | —                    |
| Later roll    | slip threshold down | tractionX threshold up   | `tractionX` down  | —                    |
| Stronger roll | roll `txScale` up   | roll `tyScale` down      | —                 | —                    |
| Weaker roll   | roll `txScale` down | roll `tyScale` up        | —                 | —                    |

---

## One-line summary

* **Hook earlier/later** = mostly `STRIBECK_B`
* **Hook stronger/weaker** = mostly hook `tyScale`
* **Roll earlier/later** = mostly roll thresholds
* **Roll stronger/weaker** = mostly roll `txScale` and `tyScale`





