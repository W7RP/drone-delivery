# Marker detector output for the log: the landing target it sends to PX4.
uint64 t_us
uint16 marker_id
# True when this frame contained the marker.
bool detected
# Detections so far for this marker; the target is sent once there are enough.
uint32 n_detections
# Spot centre (marker centre minus its configured offset), PX4 local NED, metres.
float32[3] target_ned_m
# Marker side in pixels in this frame (0 when not detected).
float32 marker_px
