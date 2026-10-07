# One message of docs/spec.md 4.2 as exchanged in SITL. Each message name has its own topic,
# /dd/msg/<name in lower case>. Fields a message does not carry keep the NONE values below.

uint16 NONE_ID16 = 65535
uint32 NONE_ID32 = 4294967295
uint8 NONE_CODE = 255

# Spec message name, e.g. LANDING_REQUEST.
string name
# Per sender, +1 for every new message, unchanged on a re-send (spec 4.1).
uint32 msg_seq
# Sim time of sending in microseconds (Gazebo clock, which PX4 SITL runs in lockstep with).
uint64 sent_t_us
# DRONE, PAD or FLEET_MANAGER.
string sender

uint16 drone_id
uint16 pad_id
uint32 pod_id
uint32 order_id
uint32 pickup_order_id
uint16 dest_pad
# Spot code (schemas/common.schema.json Spot): 0 IN, 1 OUT, 2 PARKING.
uint8 spot
# Fraction of battery capacity; NaN when the message carries none.
float32 soc
uint8 attempt
# RELEASE_REPORT latch_open, LATCH_REPORT locked.
bool flag
# Reason, outcome, mission or mode as the spec's enum name, empty when none.
string code
float32 dwell_s
uint16 queue_pos
