| | node_name      | string  | -              |   -           | Yes                         | set the name for ROS node                                         | must not start with a leading '/' |
| | topic_name     | string  | -              |   -           | Yes                         | set the name for ROS topic                                        | must start with a leading '/' |
| | msgs_name      | string  | -              |   -           | No                          | set the base name for the topics and interfaces                   | If it is not specified, the control related topics and services will not be initialized |
| | period         | double  | s              |   0.02        | No                          | refresh period of the broadcasted values in s                     | optional, default 20ms |