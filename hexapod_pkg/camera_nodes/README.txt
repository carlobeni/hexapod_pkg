============================================================
CAMERA_NODES — WEBCAM IMAGE PUBLISHER
============================================================

FILE:
-----
web_camera_node.py

NODE NAME:
----------
webcam_node


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

This node publishes real-time images from a local V4L2-compatible
webcam into ROS 2.

It captures raw frames using OpenCV, converts them to ROS Image
messages, and publishes them at a fixed frame rate with minimal
latency.


------------------------------------------------------------
INPUT / OUTPUT
------------------------------------------------------------

INPUT:
------
- Local camera device (/dev/videoX)

OUTPUT:
-------
- sensor_msgs/Image (BGR8 encoded)


------------------------------------------------------------
DESIGN PRINCIPLES
------------------------------------------------------------

- Acquisition only (no processing)
- Low latency over reliability
- Deterministic frame rate
- Downstream nodes handle all vision logic


------------------------------------------------------------
PARAMETERS
------------------------------------------------------------

- topic_image (string)  
  Output ROS topic for the image stream.

- camera_index (int)  
  Selects the camera device:
      0 → /dev/video0
      1 → /dev/video1
      ...


------------------------------------------------------------
MESSAGE FORMAT
------------------------------------------------------------

- Encoding: bgr8
- Header timestamp: current ROS time
- Frame ID: camera_optical_frame


------------------------------------------------------------
QUALITY OF SERVICE (QoS)
------------------------------------------------------------

- qos_profile_sensor_data
- Best-effort delivery
- Optimized for high-rate sensor streams


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Pure sensor source.

No filtering, no inference, no interpretation.
Only image acquisition and publication.


------------------------------------------------------------
END OF FILE
------------------------------------------------------------
