============================================================
AI MODELS FOLDER — OVERVIEW
============================================================

This folder contains all machine learning models used by
the vision nodes of the system.

These models are NOT standalone:
- they are loaded at runtime by ROS 2 vision nodes
- they provide perception capabilities (objects, gestures)
- they work identically in simulation and on the real robot

The robot itself does not “see”.
Vision nodes + models = perception.


------------------------------------------------------------
TECHNOLOGIES USED
------------------------------------------------------------

YOLO (You Only Look Once):
- real-time object detection framework
- single forward pass → bounding boxes + classes
- optimized for speed and edge devices
- used here via YOLOv8 (Ultralytics)

MediaPipe:
- Google framework for perception pipelines
- provides optimized landmark detection models
- used here for hand gesture recognition
- works on RGB images without depth sensors


------------------------------------------------------------
MODELS INCLUDED
------------------------------------------------------------


1) cubitos.pt
------------------------------------------------------------
- Type: YOLOv8n custom model
- Training: Roboflow + Google Colab
- Dataset: custom, created by the author
- Classes: 
  - "cube" (single class)

Usage:
- cube detection
- target following
- simple object-based navigation


2) pelotitas.pt
------------------------------------------------------------
- Type: YOLOv8n custom model
- Training: Roboflow + Google Colab
- Dataset: custom, created by the author
- Classes:
  - "black ball"
  - "blue ball"
  - "red ball"
  - "yellow ball"

Usage:
- multi-class object detection
- color-based behaviors
- swarm and target differentiation


3) hand_landmarker.task
------------------------------------------------------------
- Type: MediaPipe Hand Landmark model
- Source: official Google release
- Training: not custom (pretrained)

Usage:
- hand landmark extraction
- gesture recognition
- social robot interaction (human → robot commands)


