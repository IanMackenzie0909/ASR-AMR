# True only when the detector has a usable target for the controller.
bool target_visible

# Normalized horizontal target error; positive means the target is left of center.
float32 yaw

# Target distance in meters; use -1.0 when depth is not valid.
float32 depth

# Detector confidence for the selected target; 0.0 when no target is valid.
float32 confidence

# Detector class id for the selected target; -1 when no target is valid.
int32 class_id
