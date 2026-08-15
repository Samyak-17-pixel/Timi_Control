# Qualification gate detection output (two orange vertical poles required for valid center).
bool pole1_detected
bool pole2_detected
bool gate_center_valid
# Horizontal center of gate opening in image pixels (image x, left=0). NaN if invalid.
float32 gate_center_x_px
# 0 = UNKNOWN (no valid center), 1 = ALIGNED, 2 = NOT_ALIGNED
uint8 alignment_status
# gate_center_x_px - camera_center_x_px (positive = gate to the right of image center)
int32 center_error_px
