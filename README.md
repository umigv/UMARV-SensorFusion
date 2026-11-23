# UMARV-SensorFusion

The active RTL contains an [FP32 fused multiply-accumulate lane](FP32_MAC_DESIGN.md) and a [shared iterative FP32 division and square-root unit](FP32_DIVSQRT_DESIGN.md). Each is verified independently.

Run `make test synth-check` in this directory with Python 3, Icarus Verilog, and a GCC-compatible compiler.

The previous fixed-point design and its documentation are preserved on the `fixed-point-baseline` Git branch. Project planning is in the [team document](https://docs.google.com/document/d/1aGUzmQvTQ0tSf5vLGLYOxvFpVVRiFReiAoUfX6Jszvw/edit?tab=t.t4ux1nkxj3xk).
