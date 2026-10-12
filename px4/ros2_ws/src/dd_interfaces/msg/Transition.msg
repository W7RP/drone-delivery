# A state-machine transition of the spec (visit rows V.., drone rows D..), as executed in SITL.
uint64 t_us
# VISIT (pad controller) or DRONE (drone agent).
string machine
# Spec row ID, e.g. V06 or D04.
string row
uint16 pad_id
uint16 drone_id
# Spec state names (VisitState or DroneMode); "(none)" for a visit being created.
string from_state
string to_state
