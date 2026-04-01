# pi0 Policy HTTP Protocol

This document defines the current HTTP contract between `abb_pi0_bridge` and a future pi0/openpi policy service.

The goal is to keep the policy side simple:

- The bridge sends ABB robot observations.
- The policy returns the next ABB joint-position target.
- The bridge keeps responsibility for safety filtering and output gating.

## Endpoint

- Method: `POST`
- Path: `/infer`
- Content type: `application/json`

Optional health endpoint:

- Method: `GET`
- Path: `/healthz`

## Request schema

```json
{
  "api_version": "2026-03-30",
  "control_mode": "streaming",
  "robot": {
    "vendor": "ABB",
    "model": "abb_irb6700",
    "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
    "action_space": "joint_position"
  },
  "observation": {
    "stamp_sec": 1710000000.123,
    "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
    "joint_positions": [0.0, 0.1, -0.2, 0.0, 0.3, 0.0],
    "joint_velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "joint_efforts": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}
```

## Response schema

The bridge currently accepts any one of these equivalent forms:

### Preferred

```json
{
  "joint_positions": [0.0, 0.1, -0.2, 0.0, 0.3, 0.0]
}
```

### Also accepted

```json
{
  "joint_positions": {
    "joint_1": 0.0,
    "joint_2": 0.1,
    "joint_3": -0.2,
    "joint_4": 0.0,
    "joint_5": 0.3,
    "joint_6": 0.0
  }
}
```

```json
{
  "action": {
    "joint_positions": [0.0, 0.1, -0.2, 0.0, 0.3, 0.0]
  }
}
```

## Semantics

- The policy output is interpreted as the next ABB 6-axis joint-position target.
- The bridge reorders dict outputs into the ABB joint order from the observation.
- The bridge rejects malformed vectors.
- The bridge applies joint-limit clamping and per-step motion limiting after the response arrives.
- The bridge may fall back to `mock_hold` if the policy request fails and fallback is enabled.

## What the policy server does not need to do

The policy server does not need to:

- know ROS message types,
- know the ABB ROS2 controller topic,
- perform low-level safety clamping,
- arbitrate with `click_to_move`.

Those responsibilities stay inside `abb_pi0_bridge`.

## Minimal server behavior

A valid minimal server can simply echo current positions:

```json
{
  "joint_positions": [0.0, 0.1, -0.2, 0.0, 0.3, 0.0]
}
```

That is exactly what the local `pi0_policy_stub_server` does for safe end-to-end testing.
