string UNKNOWN = UNKNOWN
string VERY_UNLIKELY = VERY_UNLIKELY
string UNLIKELY = UNLIKELY
string POSSIBLE = POSSIBLE
string LIKELY = LIKELY
string VERY_LIKELY = VERY_LIKELY

google_cloud_vision_msgs/BoundingPoly bounding_poly
google_cloud_vision_msgs/BoundingPoly fd_bounding_poly
google_cloud_vision_msgs/Landmark[] landmarks
float32 roll_angle
float32 pan_angle
float32 tilt_angle
float32 detection_confidence
float32 landmarking_confidence
string joy_likelihood
string sorrow_likelihood
string anger_likelihood
string surprise_likelihood
string under_exposed_likelihood
string blurred_likelihood
string headwear_likelihood