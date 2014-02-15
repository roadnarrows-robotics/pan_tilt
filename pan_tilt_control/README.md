Pan-Tilt Control Package
=============

The *pan_tilt_control* node provides the core functionality for the **ROS**
framework to interface with the pan-tilt robotic mechanism. The
*pan_tilt_control* node runs on target as a deamon process
(along with roscore).

In addition to *pan_tilt_control*, two fundamental python application are also
included with this package:
* *pt_panel* - a GUI application that provides basic tan-tilt control and
monitoring.
* *pt_teleop* - a command-line application that provides teleoperation of the
tan-Tilt through an Xbox360 controller.
