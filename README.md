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

```

# Startup
```text
cd fileholder
flutter run
```
# Alternatively:
```text
flutter run --windows
flutter run --linux
flutter run --ios
flutter run --android
flutter run --macos
flutter run --web
```
# Known errors

No real-life tests yet. If you have input, please contribute here, especially if you have experience with specific bowling patterns. tuning parameters need help bellow!

# Model tweaks are as follows.

## Lane Breakdown Settings

| Mode           | OIL_PICKUP_RATE | OIL_DEPOSIT_RATE | OIL_CARRY_BLEND | Effect                                                     |
| -------------- | --------------- | ---------------- | --------------- | ---------------------------------------------------------- |
| **Slow**       | 0.020           | 0.012            | 0.25            | Minimal transition, lane stays stable longer               |
| **Medium**     |  0.36           | 0.20             | 1.7             | Balanced, realistic breakdown after full round             |
| **Fast**       | 0.72            | 0.4              | 3.4             | Faster burn, noticeable carrydown                          |
| **Aggressive** | 2.88            | 1.6              | 13.6            | Rapid transition, strong carrydown, backend gets sensitive |

---


# tuning parameters

## Bowling sim tuning guide (current physics)

This matches the **stateful traction + Stribeck/Hersey model**.

The rule is simple:

* **priority1** = first knob to try
* **priority2** = second knob if priority1 is not enough
* **priority3** = shaping knob
* **priority4** = last-resort trigger adjustment

Do **not** start with thresholds unless the earlier priorities are already close.

---

## Earlier hook

```dart
const double STRIBECK_B = 0.010;
// priority1 → change this first
// main hook timing control
// lower B = friction ramps up sooner out of oil

const double STRIBECK_A = 0.0024;
// priority2 → change this second
// gives friction something to build from
// without enough base friction, the ball cannot read early

const double TY_RISE = 1.2;
// priority3 → change this third
// builds lateral traction faster
// higher = earlier hook development

const double OIL_Y_DROP = 0.75;
// priority4 → change this fourth
// reduces how much oil suppresses hook
// lower = hook survives oil better

// last resort
tractionY > 0.18 → 0.16
// lowers hook trigger threshold
// only change after physics are correct
```

---

## Later hook

```dart
const double STRIBECK_B = 0.010;
// priority1 → change this first
// higher = friction ramps later

const double STRIBECK_A = 0.0024;
// priority2 → change this second
// lower = less early friction

const double TY_RISE = 1.2;
// priority3 → change this third
// lower = slower hook buildup

const double OIL_Y_DROP = 0.75;
// priority4 → change this fourth
// higher = oil suppresses hook more

// last resort
tractionY > 0.18 → 0.21
```

---

## Stronger hook

```dart
const double TY_RISE = 1.2;
// priority1 → change this first
// main hook strength knob in new system
// higher = more lateral force buildup

const double OIL_Y_DROP = 0.75;
// priority2 → change this second
// lower = hook survives oil better

const double STRIBECK_A = 0.0024;
// priority3 → change this third
// raises friction across transition

muMax * 0.72 → 0.78
// priority4 → change this fourth
// raises total friction ceiling
```

---

## Weaker hook

```dart
const double TY_RISE = 1.2;
// priority1 → change this first
// lower = weaker hook buildup

const double OIL_Y_DROP = 0.75;
// priority2 → change this second
// higher = oil kills hook more

const double STRIBECK_A = 0.0024;
// priority3 → change this third
// lowers friction availability

muMax * 0.72 → 0.66
// priority4 → change this fourth
```

---

## Earlier roll

```dart
const double ROLL_SLIP_THRESH = 0.060;
// priority1 → change this first
// higher = roll happens sooner

const double ROLL_TX_THRESH = 0.50;
// priority2 → change this second
// lower = easier to enter roll

const double TX_RISE = 2.6;
// priority3 → change this third
// builds forward traction faster

const double OIL_X_DROP = 0.35;
// priority4 → change this fourth
// lower = forward traction survives oil better
```

---

## Later roll

```dart
const double ROLL_SLIP_THRESH = 0.060;
// priority1 → change this first
// lower = roll happens later

const double ROLL_TX_THRESH = 0.50;
// priority2 → change this second
// higher = harder to enter roll

const double TX_RISE = 2.6;
// priority3 → change this third
// lower = slower forward traction buildup

const double OIL_X_DROP = 0.35;
// priority4 → change this fourth
// higher = oil suppresses forward traction more
```

---

## Stronger roll

```dart
const double TX_RISE = 2.6;
// priority1 → change this first
// increases forward drive

const double ROLL_SLIP_THRESH = 0.060;
// priority2 → change this second
// easier roll entry = more forward motion

const double MIN_LATERAL_TRACTION = 0.02;
// priority3 → change this third
// lower = cleaner forward roll (less sideways hang)
```

---

## Weaker roll

```dart
const double TX_RISE = 2.6;
// priority1 → change this first
// lower = weaker forward drive

const double ROLL_TX_THRESH = 0.50;
// priority2 → change this second
// harder to enter roll

const double MIN_LATERAL_TRACTION = 0.02;
// priority3 → change this third
// higher = more sideways continuation
```

---

## What each primary knob is for

### Hook timing

Use `STRIBECK_B` first.

* lower = earlier hook
* higher = later hook

---

### Hook strength

Use `TY_RISE` first.

* higher = stronger hook
* lower = weaker hook

---

### Roll timing

Use roll thresholds first.

* higher slip threshold = earlier roll
* lower tractionX threshold = earlier roll
* opposite = later roll

---

### Roll strength

Use `TX_RISE` first.

* higher = stronger forward roll
* lower = weaker roll

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

| Goal          | First change            | Second change         | Third change        | Last change          |
| ------------- | ----------------------- | --------------------- | ------------------- | -------------------- |
| Earlier hook  | `STRIBECK_B` down       | `STRIBECK_A` up       | `TY_RISE` up        | lower hook threshold |
| Later hook    | `STRIBECK_B` up         | `STRIBECK_A` down     | `TY_RISE` down      | raise hook threshold |
| Stronger hook | `TY_RISE` up            | `OIL_Y_DROP` down     | `STRIBECK_A` up     | friction cap up      |
| Weaker hook   | `TY_RISE` down          | `OIL_Y_DROP` up       | `STRIBECK_A` down   | friction cap down    |
| Earlier roll  | `ROLL_SLIP_THRESH` up   | `ROLL_TX_THRESH` down | `TX_RISE` up        | `OIL_X_DROP` down    |
| Later roll    | `ROLL_SLIP_THRESH` down | `ROLL_TX_THRESH` up   | `TX_RISE` down      | `OIL_X_DROP` up      |
| Stronger roll | `TX_RISE` up            | easier roll entry     | lower lateral carry | —                    |
| Weaker roll   | `TX_RISE` down          | harder roll entry     | more lateral carry  | —                    |

---

## One-line summary

* **Hook earlier/later** = mostly `STRIBECK_B`
* **Hook stronger/weaker** = mostly `TY_RISE`
* **Roll earlier/later** = mostly roll thresholds
* **Roll stronger/weaker** = mostly `TX_RISE`

# defaluts
```dart
// ─── STRIBECK / FRICTION ─────────────────────────────
const double STRIBECK_A = 0.0024;
const double STRIBECK_B = 0.010;
const double STRIBECK_C = 0.005;
const double STRIBECK_D = 12.0;
const double H_REF = 2.0e-6;

// ─── OIL SUPPRESSION ─────────────────────────────────
const double OIL_X_DROP = 0.35;
const double OIL_Y_DROP = 0.75;
const double OIL_X_EXP  = 1.10;
const double OIL_Y_EXP  = 1.25;

// ─── TRACTION RESPONSE ───────────────────────────────
const double TX_RISE = 2.6;
const double TX_FALL = 7.0;
const double TY_RISE = 1.2;
const double TY_FALL = 6.5;

// ─── PHASE THRESHOLDS ───────────────────────────────
const double ROLL_SLIP_THRESH = 0.060;
const double ROLL_TX_THRESH   = 0.50;

// ─── FRICTION BLENDING ──────────────────────────────
const double MIN_LATERAL_TRACTION = 0.02;

---






