# ROS-Http Streamer

https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3

## Dependencies
* [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
```
$ roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```
* [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/kinetic) --branch kinetic

## Issues
```
File "/home/rapsealk/catkin_ws/src/rosbridge_suite/rosbridge_server/scripts/rosbridge_websocket", line 40, in <module>
    from tornado.ioloop import IOLoop
ImportError: No module named tornado.ioloop
---
$ python -m pip install tornado
```

```
File "/home/rapsealk/catkin_ws/src/rosbridge_suite/rosbridge_server/src/rosbridge_server/websocket_handler.py", line 36, in <module>
    from rosauth.srv import Authentication
ImportError: No module named rosauth.srv
---
catkin_wd/src$ git clone https://github.com/GT-RAIL/rosauth.git
```

```
File "/home/rapsealk/catkin_ws/src/rosbridge_suite/rosbridge_library/src/rosbridge_library/util/__init__.py", line 19, in <module>
    import bson
ImportError: No module named bson
---
$ python -m pip install bson
$ python -m pip install pymongo
```

## Tips
```
import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)
```