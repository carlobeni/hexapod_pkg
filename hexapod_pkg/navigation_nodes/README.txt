============================================================
NAVIGATION_NODES — TECHNICAL OVERVIEW
============================================================

This folder contains high-level navigation nodes responsible for
transforming abstract perception data (occupancy grids, targets,
proximity sensors) into discrete robot motion commands.

These nodes do NOT perform perception themselves.
They consume:
- Occupancy grids
- Position and heading estimates
- Proximity and reflex sensors

And output:
- Symbolic motion commands (String-based control)

The folder defines three distinct navigation paradigms:
1) Reactive object following
2) Swarm navigation with evasion logic
3) Goal-oriented navigation with multi-sensor obstacle avoidance


------------------------------------------------------------
COMMON ARCHITECTURAL PRINCIPLES
------------------------------------------------------------

- Input abstraction is always an Int8 occupancy grid (NxN)
- Motion output is symbolic (e.g. "forward", "turn_left")
- Control is reactive, not trajectory-based
- Navigation logic is explicitly state-driven
- Priority is given to safety reflexes over goal pursuit

Philosophy:
    Perception → Abstraction → Decision → Actuation

No continuous control.
No path planners.
Only deliberate, interpretable logic.


============================================================
1. follower_navigation_node.py
(Ball Follower Navigation)
============================================================

PURPOSE
-------
Implements a minimal reactive behavior to follow the closest
detected object (typically a ball) using an occupancy grid.

This is the simplest navigation mode and serves as a baseline
reactive controller.

INPUTS
------
- Obstacle occupation grid (Int8MultiArray)

OUTPUTS
-------
- Robot command topic (String)

CORE LOGIC
----------
1) Scan grid rows from nearest to farthest
2) Identify the closest occupied row
3) Compute the average column of that object
4) Compare object column to grid center
5) Generate motion command:
   - forward
   - turn_left
   - turn_right

If no object is detected, the robot executes a predefined
search behavior (default: turn_left).

KEY PARAMETERS
--------------
- center_tol_cols:
  Deadband around the center column to avoid oscillations

- search_cmd:
  Command issued when no object is detected

CHARACTERISTICS
---------------
- Fully reactive
- No memory
- No obstacle avoidance
- Ideal for controlled environments

This node answers a single question:
    “Where is the closest object relative to my center?”


============================================================
2. swarm_navigation_with_obstacle_avoidance.py
(Swarm Navigation with Evasion)
============================================================

PURPOSE
-------
Extends the follower concept to swarm scenarios by introducing:
- Spatial averaging
- Proximity-based danger detection
- Forced evasion after close encounters

This node assumes multiple agents operating in the same space,
where collisions or object capture must be actively avoided.

INPUTS
------
- Obstacle occupation grid (Int8MultiArray)

OUTPUTS
-------
- Robot command topic (String)

PERCEPTION MODEL
----------------
- Column centroid is computed using weighted column densities
- Near-field danger zone is defined by lower grid rows
- Proximity is inferred from the number of active cells

BEHAVIOR STATES
---------------
1) NORMAL FOLLOW
   - Uses centroid column to align and move forward

2) SEARCH
   - Activated when no detections are present

3) EVASION (FORCED ROTATION)
   - Triggered when an object enters the danger zone
   - Robot ignores detections for a fixed time window
   - Executes a forced spin to disengage

KEY PARAMETERS
--------------
- danger_rows_count:
  Number of closest rows considered dangerous

- min_active_cells_proximity:
  Threshold to trigger evasion

- evasion_duration_cycles:
  Time window (in control cycles) of forced rotation

DESIGN INTENT
-------------
Prevent deadlocks, clustering, or repeated engagement with the
same object in multi-robot environments.

This node introduces the concept of:
    “Intentional blindness for survival.”


============================================================
3. target_navigation_with_obstacle_avoidance.py
(Target-Based Navigation with Multi-Sensor Avoidance)
============================================================

PURPOSE
-------
Navigate the robot toward a GPS-defined target while actively
avoiding obstacles using a fusion of:
- Occupancy grid
- Ultrasonic range
- Lateral IR reflex sensors

This is the most advanced navigation mode.

INPUTS
------
- Target GPS (PointStamped)
- Local XY position estimate
- Heading estimate
- Occupancy grid
- Ultrasonic distance
- Left and right IR binary sensors

OUTPUTS
-------
- Robot command topic (String)

COORDINATE MODEL
----------------
- Target GPS is converted to local ENU (XY) coordinates
- Navigation is performed entirely in local space
- Heading errors are computed in degrees

STATE MACHINE
-------------
Primary states:
- NAV:
  Normal goal-directed navigation

- AVOID_LEFT / AVOID_RIGHT:
  Obstacle evasion by rotation

- RECOVER:
  Smooth re-alignment to goal heading after avoidance

- CRAB_LEFT / CRAB_RIGHT:
  Lateral escape using IR reflexes (absolute priority)

SENSOR PRIORITY (HIGHEST → LOWEST)
---------------------------------
1) Lateral IR sensors (instant reflex)
2) Occupancy grid frontal inflation
3) Ultrasonic proximity
4) Goal heading alignment

OBSTACLE DETECTION
------------------
An obstacle is detected if:
- Occupancy grid cells are active in an inflated frontal region
- OR ultrasonic distance is below a safety threshold

AVOIDANCE STRATEGY
------------------
- Select avoidance direction based on grid density
- Rotate until frontal space clears
- Introduce a heading offset (clearance angle)
- Gradually recover original goal direction

KEY PARAMETERS
--------------
- heading_tol:
  Acceptable angular error

- dist_tol:
  Goal proximity threshold

- clearance_angle:
  Angular offset applied after avoidance

- inflate_rows / inflate_cols:
  Grid inflation parameters for safety margin

DESIGN PHILOSOPHY
-----------------
This node behaves like a layered reflex system:

    Reflex > Avoidance > Recovery > Goal pursuit

At no point is safety sacrificed for optimality.


============================================================
FINAL REMARKS
============================================================

The navigation_nodes folder represents a deliberate rejection of
black-box planning in favor of:

- Explicit logic
- Clear state transitions
- Sensor priority hierarchies
- Predictable behavior

------------------------------------------------------------
END OF FILE
------------------------------------------------------------