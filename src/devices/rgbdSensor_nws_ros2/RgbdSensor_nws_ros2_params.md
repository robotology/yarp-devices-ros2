| | period            | double  | s   |  0.02  | No                             | refresh period of the broadcasted values in s                              |                               |
| | node_name         | string  | -   |  -     | Yes                            | name of the ros2 node                                                      |                               |
| | color_topic_name  | string  | -   |  -     | Yes                            | ros rgb topic (it's also the base name for the rgb camera_info topic)      | must start with a leading '/' |
| | depth_topic_name  | string  | -   |  -     | Yes                            | ros depth topic (it's also the base name for the depth camera_info topic)  | must start with a leading '/' |
| | force_info_synch  | int     | -   |  0     | No                             | if 1, it forces synching images time stamp with and cameras time stamp     |                               |
| | depth_frame_id    | string  | -   |  -     | Yes                            | The depth image frame                                                      |                               |
| | color_frame_id    | string  | -   |  -     | Yes                            | The color image frame                                                      |                               |
