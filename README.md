# bowling
Real Physics Bowling Simulation, simulates ball on real complex bowling pattern based on actual physics. 

feel free to contribute to this code, its quite thurough just needs a few tweaks

# known errors
cant push someone all the way to their left
brush isnt having any effect yet. 

## Lane Breakdown Settings

| Mode           | OIL_PICKUP_RATE | OIL_DEPOSIT_RATE | OIL_CARRY_BLEND | Effect                                                     |
| -------------- | --------------- | ---------------- | --------------- | ---------------------------------------------------------- |
| **Slow**       | 0.020           | 0.012            | 0.25            | Minimal transition, lane stays stable longer               |
| **Medium**     | 0.030           | 0.018            | 0.35            | Balanced, realistic breakdown                              |
| **Fast**       | 0.050           | 0.030            | 0.50            | Faster burn, noticeable carrydown                          |
| **Aggressive** | 0.055           | 0.035            | 0.60            | Rapid transition, strong carrydown, backend gets sensitive |

---

### Quick Guide

* Increase **OIL_PICKUP_RATE** → fronts burn faster
* Increase **OIL_DEPOSIT_RATE** → more oil moves downlane
* Increase **OIL_CARRY_BLEND** → carrydown has stronger impact
