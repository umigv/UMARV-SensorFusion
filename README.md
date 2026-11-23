# UMARV-SensorFusion

The active RTL target is one FP32 fused multiply-accumulate lane. The [MAC design and verification guide](FP32_MAC_DESIGN.md) describes its interface, rounding, exceptions, and build commands.

Run `make test synth-check` in this directory with Python 3 and Icarus Verilog.

The previous fixed-point RTL and testbenches are in [legacy/fixed-point](legacy/fixed-point). The `fixed-point-baseline` Git branch records the working tree from before the migration. The [four-state SR-UKF](SRUKF_MIXED_PRECISION.md) and [five-state dead-wheel bias estimator](SRUKF_BIAS.md) retain their architecture and verification documentation. Project planning is in the [team document](https://docs.google.com/document/d/1aGUzmQvTQ0tSf5vLGLYOxvFpVVRiFReiAoUfX6Jszvw/edit?tab=t.t4ux1nkxj3xk).
