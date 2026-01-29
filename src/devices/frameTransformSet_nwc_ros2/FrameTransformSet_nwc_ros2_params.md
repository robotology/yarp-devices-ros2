| GENERAL | refresh_interval  | double  | seconds | 0.1         | No   | The time interval outside which timed ft will be deleted                                              |
| GENERAL | period            | double  | seconds | 0.01        | No   | The PeriodicThread period in seconds                                                                  |
| GENERAL | asynch_pub        | int     | -       | 1           | No   | If 1, the fts will be published not only every "period" seconds but also when set functions are called|
| ROS2    | ft_node           | string  | -       | tfNodeSet   | No   | The name of the ROS2 node                                                                             |
| ROS2    | namespace         | string  | -       |   -         | No   | optional namespace for ros2 node                                                                       |                                  |
| ROS2    | ft_topic          | string  | -       | /tf         | No   | The name of the ROS2 topic from which fts will be received                                            |
| ROS2    | ft_topic_static   | string  | -       | /tf_static  | No   | The name of the ROS2 topic from which static fts will be received                                     |
