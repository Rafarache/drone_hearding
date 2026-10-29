# Image acquisition time, frame_id="map". One message per processed camera image.
std_msgs/Header header
uint32 source_drone_index
# Image-time planar camera pose, using the detector's existing projection origin.
geometry_msgs/Pose camera_pose
# Near range limits clearing only; valid detections inside it can still add points.
float64 near_range_m
float64 far_range_m
float64 horizontal_fov_rad
# Empty means a successfully processed frame with no cow detections.
cow_mapping_interfaces/CowDetection[] detections
