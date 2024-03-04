std_msgs/Header header
string unique_id
string type

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
bool person_data
int64 color_person
yolov8_msgs/KeyPoint2D[] skeleton2d
yolov8_msgs/KeyPoint3D[] skeleton3d
int8 pointing_direction

# object
bool object_data
int8[] color_object
bool collision

sensor_msgs/Image image
