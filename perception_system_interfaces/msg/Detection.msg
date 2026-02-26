std_msgs/Header header
string unique_id
string type
bool collision

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
int64 color_person
yolo_msgs/KeyPoint2D[] skeleton2d
yolo_msgs/KeyPoint3D[] skeleton3d
int8 pointing_direction
int8 body_pose

# object
int8[] color_object

sensor_msgs/Image image
