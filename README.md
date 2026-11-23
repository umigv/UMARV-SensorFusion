# UMARV-SensorFusion

The active RTL contains an [FP32 fused multiply-accumulate lane](FP32_MAC_DESIGN.md) and a [shared iterative FP32 division and square-root unit](FP32_DIVSQRT_DESIGN.md). Each is verified independently.

Run `make test synth-check` in this directory with Python 3, Icarus Verilog, and a GCC-compatible compiler.

The previous fixed-point RTL and testbenches are in [legacy/fixed-point](legacy/fixed-point). The `fixed-point-baseline` Git branch records the working tree from before the migration. The [four-state SR-UKF](SRUKF_MIXED_PRECISION.md) and [five-state dead-wheel bias estimator](SRUKF_BIAS.md) retain their architecture and verification documentation. Project planning is in the [team document](https://docs.google.com/document/d/1aGUzmQvTQ0tSf5vLGLYOxvFpVVRiFReiAoUfX6Jszvw/edit?tab=t.t4ux1nkxj3xk).
