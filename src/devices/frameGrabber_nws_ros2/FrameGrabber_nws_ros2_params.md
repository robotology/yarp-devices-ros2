| | period         | double  | s              |   0.02        | No                          | refresh period of the broadcasted values in s                     | optional, default 20ms |
| | node_name      | string  | -              |   -           | Yes                         | set the name for ROS node                                         | must not start with a leading '/' |
| | namespace      | string  | -              |   -           | No                          | optional namespace for ros2 node                                   |                                  |
| | topic_name     | string  | -              |   -           | Yes                         | set the name for ROS topic                                        | must start with a leading '/' |
| | frame_id       | string  | -              |   -           | Yes                         | set the name for the reference frame                              | must not start with a leading '/' |
