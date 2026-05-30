# Bowling

https://bowling-ecru.vercel.app/


Real Physics Bowling Simulation. It simulates ball movement on a complex bowling pattern based on actual physics.

Feel free to contribute to this code. It is already fairly thorough, but still needs a few tweaks.

# FORMULA OPTIONS 
```text
BOWLING2&3: μ_eff = oil · [MU_OIL_MIN + C(1-e^(-D·Ĥ))]  +  (1-oil) · [μ_min + a/(H+b) + c·H]
BOWLING4: μ(v_slip, H) = μ_c + (μ_s·e^(-aH) - μ_c) · e^(-(v_slip/v_s(1+bH))²)

      CURRENT bowling5built (physics.dart):

        ┌─ Stribeck ──────────────────────────────────────┐
        │ muBase = muMin + A/(hHat+B) + C(1−e^(−D·hHat))  │
        └─────────────────────────────────────────────────┘
             │
             ├─ muK = clamp(muBase · 0.4,  ROLL_RES, muDry)
             ├─ muS = clamp(muBase,         muK,     muDry)
             │
        ┌─ Slip blend ────────────────────────────────────────────┐
        │ slipBlendX = (X_SKID_SLIP  − slipRatio)                 │
        │            / (X_SKID_SLIP  − X_ROLL_SLIP)   → [0,1]     │
        │                                                         │
        │ oilGripX   = 1 − OIL_X_DROP · oil^OIL_X_EXP             │
        │ tractionTargetX = slipBlendX · oilGripX                 │
        └─────────────────────────────────────────────────────────┘
             │
        ┌─ Traction memory ─────────────────────────────┐
        │ dT/dt = (target − T) · rate   (rise ≠ fall)   │
        └───────────────────────────────────────────────┘
             │
        ┌─ Final blend ──────────────────────────────────┐
        │ μX = μK·(1−TX) + μS·TX                         │
        │ μY = μK·(1−TY) + μS·TY                         │
        └────────────────────────────────────────────────┘




```

## Setup

```text
//flutter config --enable-windows-desktop //enable flutter for your platform. 
//flutter create fileholder //either create your fileholder and replace lib or create later with create . 

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
flutter run -d windows
flutter run -d linux
flutter run -d ios
flutter run -d android
flutter run -d macos
flutter run -d web
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

# defaults
```dart
// ─── STRIBECK / FRICTION ─────────────────────────────
const double STRIBECK_A = 0.0080; // Fluid resistance
const double STRIBECK_B = 0.040; // Oiled friction floor
const double STRIBECK_C = 0.0150; // Transition scaling
const double STRIBECK_D = 15.0;   // Shape (10 = neutral)
const double H_REF      = 2.2e-6; // Oil depth sensitivity

// ─── OIL SUPPRESSION ─────────────────────────────────
const double OIL_X_DROP = 0.40;   // When long. friction starts to die
const double OIL_Y_DROP = 0.80;   // When lat. friction starts to die
const double OIL_X_EXP  = 1.15;   
const double OIL_Y_EXP  = 1.30;   

// ─── TRACTION RESPONSE ───────────────────────────────
const double TX_RISE = 3.5;       // Faster longitudinal grab
const double TX_FALL = 6.0;       
const double TY_RISE = 2.0;       // Stronger lateral "turn"
const double TY_FALL = 5.0;       

// ─── PHASE THRESHOLDS ───────────────────────────────
const double ROLL_SLIP_THRESH = 0.05; // Slightly tighter roll lock
const double ROLL_TX_THRESH   = 0.60; 

// ─── FRICTION BLENDING ──────────────────────────────
const double MIN_LATERAL_TRACTION = 0.03;

// brush movement how much oil gets dropped off ──────────────────────────────
  double alphaF = 0.25,
  double rF = 0.91,
  double alphaR = 0.32,
  double rR = 0.94,
  double k = 0.85,
  double beta = 0.35,
  double gamma = 0.12,
  double eta = 0.45,
  double lambda = 0.75,


---






