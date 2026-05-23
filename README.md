# ACC 2DOF Motion Simulator

A lightweight Python core for a 2 degrees-of-freedom motion simulator driven by Assetto Corsa Competizione UDP telemetry.

## Features

- Listens for ACC UDP telemetry on `0.0.0.0:9996`
- Parses local car motion data
- Calculates pitch and roll cues using orientation and g-forces
- Logs platform commands to the console or CSV file

## Requirements

- Python 3.10+

## Usage

1. Enable UDP telemetry in Assetto Corsa Competizione.
2. Run the simulator script:

```bash
python MotionSimulator.py --port 9996 --log-file motion.csv
```

3. Tune the motion cue with command-line options:

```bash
python MotionSimulator.py --pitch-scale 1.0 --roll-scale 1.0 --pitch-accel-gain 4.0 --roll-accel-gain 4.0 --max-angle 15.0
```

## Options

- `--host`: UDP listen address (default `0.0.0.0`)
- `--port`: ACC telemetry UDP port (default `9996`)
- `--pitch-scale`: Scale factor for pitch input
- `--roll-scale`: Scale factor for roll input
- `--pitch-accel-gain`: Longitudinal g-force gain for pitch
- `--roll-accel-gain`: Lateral g-force gain for roll
- `--max-angle`: Maximum output platform angle in degrees
- `--log-file`: CSV file path for command logging
- `--verbose`: Enable debug logging

## Extending for hardware

Replace `ConsoleActuatorOutput` with a real actuator driver that sends the computed pitch/roll values to your motion platform.
