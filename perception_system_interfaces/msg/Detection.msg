std_msgs/Header header
string unique_id
# 0 object 1 person
int8 type

# yolo data
int32 class_id
string class_name
float64 score

# location
geometry_msgs/Pose2D center2d
geometry_msgs/Point bbox2d
geometry_msgs/Pose center3d
geometry_msgs/Vector3 bbox3d

# person
int64 color_id
yolov8_msgs/KeyPoint2D[] skeleton2d
yolov8_msgs/KeyPoint3D[] skeleton3d

# object
int8[] color
bool collision
