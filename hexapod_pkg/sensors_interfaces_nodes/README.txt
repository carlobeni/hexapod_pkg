============================================================
SENSORS_INTERFACES_NODES — ULTRASONIC RANGE PROCESSING
============================================================

FILE:
-----
compute_ultrasonic_ranges.py

NODE NAME:
----------
compute_ultrasonic_ranges


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

This node acts as a fast and deterministic interface layer between
a raw ultrasonic sensor (Gazebo or real hardware) and the rest of
the navigation system.

Its sole responsibility is to:
- Sanitize ultrasonic range readings
- Filter invalid or "no echo" values
- Publish a clean, minimal scalar distance signal

The output is intentionally simplified to a Float32 value suitable
for reactive navigation logic.


------------------------------------------------------------
INPUT / OUTPUT
------------------------------------------------------------

INPUT:
------
- sensor_msgs/Range
  Raw ultrasonic measurement, typically coming from Gazebo or
  a hardware driver.

OUTPUT:
-------
- std_msgs/Float32
  Processed distance in meters, or -1.0 if no valid detection
  is present.


------------------------------------------------------------
DESIGN PHILOSOPHY
------------------------------------------------------------

This node deliberately avoids:
- Temporal filtering
- Averaging
- State estimation
- Probabilistic modeling

Instead, it follows a strict rule:

    “Either the measurement is usable, or it does not exist.”

The goal is to provide higher-level navigation nodes with an
unambiguous signal that can be used in reflex logic without
additional interpretation.


------------------------------------------------------------
VALIDATION MODEL
------------------------------------------------------------

The incoming range value is classified into three exclusive cases:

1) NO ECHO
-----------
If the received range is numerically close to a known sentinel
value produced by the simulator when no obstacle is detected,
the measurement is rejected.

This condition is detected using a tolerance-based comparison
to account for floating-point noise.

Output:
    -1.0

2) VALID MEASUREMENT
--------------------
If the range lies within a physically plausible interval:

    [0.02 m , 4.50 m]

the value is accepted and forwarded unchanged.

Output:
    r (meters)

3) INVALID MEASUREMENT
---------------------
Any value outside the valid interval that is not classified as
a no-echo sentinel is considered invalid.

Output:
    -1.0


------------------------------------------------------------
NUMERICAL PARAMETERS
------------------------------------------------------------

- Minimum valid distance:
      0.02 meters (2 cm)

- Maximum valid distance:
      4.50 meters

- No-echo sentinel value:
      1.4500000476837158

- No-echo tolerance:
      ±0.05

These constants reflect the behavior of the ultrasonic sensor
model used in simulation and are intentionally hard-coded to
avoid ambiguity.


------------------------------------------------------------
QUALITY OF SERVICE (QoS)
------------------------------------------------------------

QoS Profile:
------------
- Reliability: BEST_EFFORT
- History: KEEP_LAST
- Depth: 10

Rationale:
----------
Ultrasonic data is high-frequency and transient.
Losing individual samples is preferable to blocking or lagging
the control loop.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

This node serves as a **semantic adapter**:

    Raw physics signal  →  Binary-valid distance abstraction

It enables higher-level navigation nodes to implement logic such as:

- “Obstacle closer than X meters?”
- “Is frontal space clear?”
- “Ignore ultrasonic sensor if no echo exists”

without needing to know anything about sensor quirks, simulator
artifacts, or floating-point anomalies.


------------------------------------------------------------
INTENDED USAGE
------------------------------------------------------------

- Reactive obstacle avoidance
- Safety reflex triggering
- Complementary sensor to vision-based grids
- Real-time navigation loops

It is NOT intended for:
- Mapping
- Localization
- Long-range planning


------------------------------------------------------------
END OF FILE
------------------------------------------------------------
