============================================================
WORLD PLUGINS — SHORT EXPLANATION
============================================================

Gazebo does NOT simulate sensors automatically.

Each sensor (IMU, magnetometer, GPS/NavSat):
- requires its corresponding world plugin
- otherwise publishes invalid or static data

World plugins are the systems that:
- update sensor states
- generate realistic measurements
- make ROS topics meaningful

No plugin → no real sensor behavior  
Even if the sensor is declared in the robot model.


------------------------------------------------------------
SPHERICAL COORDINATES (NAVSAT)
------------------------------------------------------------

The `<spherical_coordinates>` block:
- defines the Earth reference (WGS84)
- sets the GPS origin of the world

It is mandatory for:
- NavSat sensors
- GPS-based navigation
- consistent sim ↔ real behavior

Without it, GPS data has no physical meaning.


============================================================
WORLD OVERVIEW
============================================================


------------------------------------------------------------
empty.world
------------------------------------------------------------
- Flat ground only
- Full sensor plugin stack enabled
- Earth-referenced GPS origin

Used for:
- sensor validation
- control debugging
- baseline tests


------------------------------------------------------------
ball_tracker_world.world
------------------------------------------------------------
- Flat ground + reference markers
- One colored sphere (target)
- GPS rectangle visualization

Used for:
- vision-based target following
- local motion control
- perception + actuation coupling


------------------------------------------------------------
navigation_with_box_world.world
------------------------------------------------------------
- Flat ground + many static boxes
- GPS reference polygon
- Known obstacles distribution

Used for:
- GPS navigation to target
- obstacle-aware motion
- path planning logic validation


------------------------------------------------------------
swarm_space.world
------------------------------------------------------------
- Flat ground
- Multiple spheres in geometric formation
- Color-coded targets

Used for:
- swarm / multi-agent behaviors
- relative positioning
- collective perception logic


------------------------------------------------------------
FINAL NOTE
------------------------------------------------------------

Worlds define:
- physics
- sensors
- reference frames