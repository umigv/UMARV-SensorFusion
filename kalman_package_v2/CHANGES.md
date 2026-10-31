# Cleanup and fixes

- Removed circular includes that exposed prototypes before their types existed.
  Public, internal, math and workspace headers now have one-way dependencies.
- Added direct standard/project includes in implementation files; normalized
  final newlines and removed stale commentary.
- Implemented the declared-but-missing `srukf_reset` and `srmat_is_finite`;
  reused the latter in Cholesky input validation.
- Removed unused workspace buffers; retained only actual scratch allocations.
- Replaced sscanf parsing with strict strtod token parsing. The former parser
  accepted `G 0 0 0 junk` because the failed optional numeric conversion left
  its assignment count at four. The CLI now consistently requires four GPS
  fields and five odometry fields, matching the uploaded code's chosen format.
- Prevented failed updates from advancing the CLI timestamp watermark.
- Restored explicit nonfinite-angle handling and used the common angle wrapper
  in CLI odometry handling.
- Added scale-relative numerical rank detection to QR; exact dependent rows
  can otherwise leave tiny nonzero pivots due to rounding.
- Bounded prediction work so huge finite timestamps cannot trigger an
  effectively endless loop. Oversized gaps fail without modifying the filter.
- Added a Makefile, example packets, usage/model documentation and regression
  tests. Kept existing state order, API function signatures, and tuning defaults.

Validation: GCC C11 with -Wall -Wextra -Werror; numerical/API and CLI tests;
repeat of the same suite with the undefined-behavior sanitizer. All passed.
