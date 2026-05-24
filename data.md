# Temporary Parameter Snapshot

Date: 2026-05-24

This file records the tuned default parameters before slowing the global turn defaults for ADV1 debugging.

## Track Defaults

| Parameter | Value |
| --- | ---: |
| `BASE` | `33.0` |
| `KP` | `60.0` |
| `KD` | `8.0` |
| `CENTER_BIAS` | `1.3` |
| `CORNER_ADVANCE_MS` | `220` |
| `RECOVER_MS` | `0` |
| `LEFT_TRIM` | `0.89` |
| `RIGHT_TRIM` | `1.00` |
| `LAPS` | `1` |

## Turn Defaults Before Slow-Turn Change

| Parameter | Value |
| --- | ---: |
| `TURN_OUT` | `60.0` |
| `TURN_IN` | `-25.0` |
| `TURN_ANGLE` | `85.0` |
| `MAX_TURN_MS` | `90000` |
| `TURN_RAMP` | `2.0` |
| `TURN_RATE_SCALE` | `1.0` |
| `TURN_RATE_KP` | `0.015` |
| `TURN_STOP_RATE` | `25.0` |
| `TURN_R0` | `25` |
| `TURN_R15` | `70` |
| `TURN_R30` | `115` |
| `TURN_R45` | `155` |
| `TURN_R60` | `190` |
| `TURN_R75` | `220` |
| `TURN_R90` | `220` |

## Gimbal / Aim Defaults

| Parameter | Value |
| --- | ---: |
| `GIMBAL_DEFAULT_SPEED_SPS` | `1000.0` |
| `GIMBAL_DEFAULT_ACCEL_SPS2` | `6000.0` |
| `GIMBAL_DEFAULT_CAL_A` | `-1.07` |
| `GIMBAL_DEFAULT_CAL_B` | `-0.05` |
| `GIMBAL_DEFAULT_CAL_C` | `0.04` |
| `GIMBAL_DEFAULT_CAL_D` | `0.91` |
| `AIM_TRACK_INTERVAL_MS` | `50` |
| `AIM_LOCK_TOLERANCE_PX` | `2` |
| `AIM_LOCK_EXIT_TOLERANCE_PX` | `4` |
| `AIM_TRACK_LOST_ERROR_MS` | `800` |

## Current Track Presets

| Preset | `BASE` | `KP` | `KD` | `CENTER_BIAS` | `CORNER_ADVANCE_MS` | `RECOVER_MS` | `LEFT_TRIM` | `RIGHT_TRIM` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `NORMAL` | `33.0` | `60.0` | `8.0` | `-0.8` | `20` | `0` | `0.89` | `1.00` |
| `ADV` | `26.0` | `50.0` | `8.0` | `-0.8` | `20` | `0` | `0.95` | `1.00` |
