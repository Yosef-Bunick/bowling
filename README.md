# bowling
Real Physics Bowling Simulation, simulates ball on real complex bowling pattern based on actual physics. 

feel free to contribute to this code, its quite thurough just needs a few tweaks

# known errors
no real life tests, if you have any input please contribute here especialy if you are experienced.

## Lane Breakdown Settings

| Mode           | OIL_PICKUP_RATE | OIL_DEPOSIT_RATE | OIL_CARRY_BLEND | Effect                                                     |
| -------------- | --------------- | ---------------- | --------------- | ---------------------------------------------------------- |
| **Slow**       | 0.020           | 0.012            | 0.25            | Minimal transition, lane stays stable longer               |
| **Medium**     | 0.18            | 0.10             | 0.85            | Balanced, realistic breakdown after full round             |
| **Fast**       | 0.36            | 0.20             | 1.7             | Faster burn, noticeable carrydown                          |
| **Aggressive** | 0.72            | 0.4              | 3.4             | Rapid transition, strong carrydown, backend gets sensitive |

---

### Quick Guide

* Increase **OIL_PICKUP_RATE** → fronts burn faster
* Increase **OIL_DEPOSIT_RATE** → more oil moves downlane
* Increase **OIL_CARRY_BLEND** → carrydown has stronger impact
