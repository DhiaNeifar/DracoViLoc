# dracoviloc_ekf

C++ extended Kalman filter for DracoViLoc direction estimates. It optionally
subscribes to `/yolo/direction` (`yolo_enabled`) and `/mobilenetv2/direction`
(`mobilenetv2_ekf_enabled`); every input and the `/ekf/direction` output uses
`geometry_msgs/Vector3Stamped` with `(x,y,z)` as a direction vector.

`/ekf_fused_target_pose` is temporarily retained as a compatibility alias and
will be removed after downstream users migrate to `/ekf/direction`.

`output_average_window` controls a causal normalized-vector average applied to
accepted EKF estimates before publication. The default is 5 samples; set it to
1 to publish the unaveraged EKF estimate.

`yolo_measurement_noise` and `mobilenetv2_measurement_noise` allow the visual
and acoustic directions to use different covariance. A value at or below zero
falls back to the common `measurement_noise`. The main bringup defaults to
0.03 for YOLO and 0.15 for MobileNetV2/ODAS because the latter is a noisier,
intermittent bearing. Sensor timestamps are kept monotonic when callbacks from
the two pipelines arrive out of order.

The EKF never subscribes to `/sst`. MobileNetV2 owns the association between
its classification result and the corresponding ODAS direction. AST and GRE
still publish `/ast/direction` and `/gre/direction` for the direct tracking
modes, but they are not EKF inputs.

## Bearing continuity gate

Acoustic inputs are protected by an EKF-aware continuity gate before the
normal measurement update. A direction is deferred when it is farther from
the predicted direction than the elapsed-time angular allowance and
statistically inconsistent according to the EKF innovation covariance. A
direction beyond the absolute angular cap is always deferred, even when the
filter is uncertain. This makes the comparison wrap-safe and lets an uncertain
filter or a rapidly moving target tolerate a moderate change without allowing
an arbitrary large jump.

The gate is enabled by default. Its defaults are a 45 degree base allowance,
a 20 degree/second motion allowance capped at 70 degrees, and five challenger
observations spanning at least 2.0 seconds. A consistent challenger is used
to reacquire the track. A gap longer than 10 seconds expires stale challenger
evidence but never makes the next arbitrary direction trusted. Recent accepted
YOLO updates prevent divergent audio from forcibly resetting a visually
anchored filter.

Parameters:

- `bearing_gate_enabled`
- `bearing_gate_base_angle_deg`
- `bearing_gate_max_angle_deg`
- `bearing_gate_max_rate_deg_s`
- `bearing_gate_confirmations`
- `bearing_gate_confirmation_time_s`
- `bearing_gate_challenger_tolerance_deg`
- `bearing_gate_reacquire_timeout_s`

Set `bearing_gate_enabled:=false` to use the original EKF behavior: only the
existing NIS innovation gate is applied. The main bringup disables the older
classifier-local fixed-angle gates while `tracking_mode:=ekf`, avoiding two
independent gates and preserving those older gates for direct tracking modes.

This is a continuity heuristic, not a reflection classifier. A stable
reflection can eventually satisfy the reacquisition criteria, and a genuine
abrupt movement outside the allowance is intentionally delayed until it has
persisted long enough to confirm.

## First-arrival authorization

`first_arrival_mode` supports `off`, `shadow`, and `enforce`. `off` preserves
the original input and EKF behavior. `shadow` logs decisions without changing
fusion. `enforce` requires a confident `/first_arrival/candidates` direction
before acoustic input can initialize the EKF or move more than 25 degrees from
the last direct-path anchor. Evidence must match the classified bearing within
20 degrees and is retained for two seconds to cover classifier latency.

Once anchored, nearby measurements still pass through the normal continuity
and NIS gates. A large challenger needs both first-arrival authorization and
the existing persistent-challenger confirmation. YOLO measurements remain
valid visual anchors. Missing or ambiguous evidence causes no acoustic update,
allowing the tracker timeout to hold the arm safely.
