## How it works

This is the sensor front end of a sensor fusion chip for a ground robot. On a fixed 400 Hz beat it

- keeps a microsecond clock,
- pulses the SyncIn pin of a VectorNav VN-300 and receives the binary message it sends back,
- reads an AMT22 absolute encoder on a free-rolling "dead wheel" over SPI,
- checks both (CRC and status for the VN-300, check bits for the encoder), and
- signals when one consistent set of measurements for that beat is ready.

A Kalman filter that uses these measurements is planned for the same chip. Until its register
interface exists, the measurements are sent out on a debug serial stream after each beat.

See [timestamper.md](timestamper.md) for the block diagram, timing and interface.

## How to test

Supply a 50 MHz clock, hold `ui[2]` (enable) high and release reset. With no sensors attached:

- `uo[0]` pulses high for 10 us every 2.5 ms,
- `uo[1]` to `uo[3]` show one 32-clock SPI read (`00 A0 00 00`) per beat,
- `uo[4]` pulses 2 ms after each beat,
- `uo[5]` and `uo[6]` carry the debug stream: while `uo[6]` is high, `uo[5]` sends the 40-bit time
  and then the measurement record, most significant bit first, one bit every 8 clocks.

## External hardware

- VectorNav VN-300, set to 921,600 baud and to send its binary output on each SyncIn pulse
- AMT222D (14-bit multi-turn AMT22) encoder on SPI
- 50 MHz oscillator
