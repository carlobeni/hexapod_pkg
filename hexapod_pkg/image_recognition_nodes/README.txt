============================================================
IMAGE_RECOGNITION_NODES — TECHNICAL OVERVIEW
============================================================

This folder contains ROS 2 nodes dedicated to visual perception,
object detection, and image-to-grid abstraction for navigation
and behavior control in a hexapod robotic system.

All nodes subscribe to a camera image topic and transform visual
information into either:
- Occupancy grids
- Distance estimations
- Debug visualizations

The nodes are divided into two main paradigms:
1) YOLO-based deep learning vision
2) Classical HSV-based computer vision


------------------------------------------------------------
1. ball_viewer_yolo_follower.py
------------------------------------------------------------

PURPOSE
-------
Detect spherical objects (balls) using a YOLO model and project
their bounding boxes into a 2D occupancy grid aligned with the
camera image.

This node is designed for follower behaviors, where detected
objects represent dynamic obstacles or targets.

MAIN FEATURES
-------------
- YOLO inference on GPU (CUDA)
- Configurable confidence threshold
- Frame skipping for performance
- Image divided into an NxN grid
- Each grid cell marked as occupied if intersected by a detection
- Optional real-time debug visualization

INPUTS
------
- Image topic (sensor_msgs/Image)

OUTPUTS
-------
- Int8MultiArray occupancy grid (row-major order)
- Debug string topic with detection summaries

NOTES
-----
- Uses a custom-trained YOLO model: pelotitas.pt
- Grid resolution is typically moderate (default: 15x15)
- Bounding boxes are directly rasterized into grid cells


------------------------------------------------------------
2. ball_viewer_yolo_swarm.py
------------------------------------------------------------

PURPOSE
-------
Same conceptual function as the follower version, but optimized
for swarm scenarios where multiple robots share perception data.

MAIN DIFFERENCES FROM FOLLOWER VERSION
--------------------------------------
- Higher grid resolution (default: 20x20)
- Lower confidence threshold
- Half-precision (FP16) inference enabled
- Tuned for higher throughput and scalability

INTENDED USE
------------
- Distributed multi-agent navigation
- Shared obstacle maps
- Collective perception systems

MODEL
-----
- Uses the same YOLO model (pelotitas.pt)


------------------------------------------------------------
3. clasic_vision_calibratiom.py
------------------------------------------------------------

PURPOSE
-------
Interactive calibration tool for classical computer vision based
on HSV color segmentation.

This node is NOT meant for autonomous operation, but for manual
tuning and visual inspection.

FUNCTIONALITY
-------------
- Live HSV threshold adjustment using OpenCV trackbars
- Real-time mask visualization
- Green cube detection via contour analysis
- Distance estimation using pinhole camera approximation

DISTANCE MODEL
--------------
distance ≈ (fx * real_object_width) / pixel_width

INPUTS
------
- Image topic

OUTPUTS
-------
- Visual only (no ROS messages published)

NOTES
-----
- Essential for determining HSV ranges before deploying
  classical vision nodes
- Acts as a human-in-the-loop calibration interface


------------------------------------------------------------
4. cube_viewer_classic_vision.py
------------------------------------------------------------

PURPOSE
-------
Detect green cubes using classical HSV segmentation and convert
the result into an occupancy grid.

This node is the classical vision counterpart of the YOLO-based
cube detector.

PIPELINE
--------
1) RGB → HSV conversion
2) Thresholding using fixed parameters
3) Morphological cleaning
4) Per-cell occupancy evaluation
5) Grid publication

ADVANTAGES
----------
- No neural networks
- Deterministic behavior
- Extremely fast and lightweight

LIMITATIONS
-----------
- Sensitive to lighting conditions
- Requires prior calibration

OUTPUT
------
- Int8MultiArray occupancy grid


------------------------------------------------------------
5. cube_viewer_yolo.py
------------------------------------------------------------

PURPOSE
-------
YOLO-based cube detection with conversion to an occupancy grid.

This node replaces classical color segmentation with a learned
model, providing higher robustness to lighting and appearance
variations.

FEATURES
--------
- GPU-accelerated YOLO inference
- Bounding box to grid projection
- Debug visualization with grid overlay
- Debug text publication

MODEL
-----
- Custom YOLO model: cubitos.pt

USE CASE
--------
- Environments with variable illumination
- When classical HSV segmentation is unreliable


------------------------------------------------------------
COMMON DESIGN PRINCIPLES
------------------------------------------------------------

- All grid-based nodes output Int8MultiArray
- Grid encoding is binary: 0 = free, 1 = occupied
- Grid is aligned with the camera image plane
- No world-frame projection is performed
- Designed to be consumed by higher-level planners or behaviors

The philosophy is explicit:
    "Vision produces abstractions, not decisions."

These nodes do not decide motion.
They only transform pixels into structured information.


------------------------------------------------------------
END OF FILE
------------------------------------------------------------
