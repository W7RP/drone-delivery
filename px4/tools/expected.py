"""The normal delivery visit as docs/spec.md defines it (3.4 visit rows, 4.3 sequence, 4.4
drone rows), restricted to what Phase 4 flies: the drone starts parked on A's OUT with a pod,
delivers it to B's IN, hops to B's OUT, latches the pod waiting there and parks. B has no
charger in SITL (spec A58), so V32 (READY) replaces V31/V33 and there is no CHARGE_START."""

# (machine, pad role, row, from, to); pad role A or B, None for the drone.
VISIT_A = [("INIT", "(none)", "READY"), ("V34", "READY", "DEPARTING"), ("V36", "DEPARTING", "CLOSED")]
VISIT_B = [
    ("V01", "(none)", "APPROACH"),
    ("V06", "APPROACH", "RELEASING"),
    ("V15", "RELEASING", "VERIFY_RELEASE"),
    ("V18", "VERIFY_RELEASE", "WAIT_OUT"),
    ("V21", "WAIT_OUT", "HOPPING"),
    ("V24", "HOPPING", "LATCHING"),
    ("V28", "LATCHING", "VERIFY_LOCK"),
    ("V32", "VERIFY_LOCK", "READY"),
]
DRONE = [
    ("D19", "PARKED", "PARKED"),
    ("D01", "PARKED", "TAKEOFF"),
    ("D02", "TAKEOFF", "EN_ROUTE"),
    ("D03", "EN_ROUTE", "HOLDING"),
    ("D04", "HOLDING", "APPROACH"),
    ("D05", "APPROACH", "ON_PAD"),
    ("D15", "ON_PAD", "ON_PAD"),      # RELEASE_CMD
    ("D15", "ON_PAD", "HOPPING"),     # HOP_CMD
    ("D15", "HOPPING", "ON_PAD"),     # HOP_DONE
    ("D15", "ON_PAD", "ON_PAD"),      # LATCH_CMD
    ("D17", "ON_PAD", "ON_PAD"),
    ("D18", "ON_PAD", "PARKED"),
]
# (sender, name, pad role or None, code or None): spec 4.2 messages in the order spec 4.3
# sends them, with this phase's start (DRONE_STATUS PARKED, PICKUP_ASSIGNED) and end (PARK).
MESSAGES = [
    ("DRONE", "DRONE_STATUS", "A", "PARKED"),
    ("FLEET_MANAGER", "PICKUP_ASSIGNED", "B", None),
    ("FLEET_MANAGER", "MISSION_ASSIGN", None, "DELIVER"),
    ("DRONE", "DEPART_REQUEST", "A", "DELIVER"),
    ("PAD", "DEPART_CLEAR", "A", None),
    ("DRONE", "DEPARTED", "A", None),
    ("DRONE", "DRONE_STATUS", None, "EN_ROUTE"),
    ("DRONE", "LANDING_REQUEST", "B", None),
    ("PAD", "LANDING_GRANT", "B", None),
    ("DRONE", "LANDED", "B", None),
    ("DRONE", "DRONE_STATUS", "B", "ON_PAD"),
    ("PAD", "RELEASE_CMD", "B", None),
    ("DRONE", "RELEASE_REPORT", "B", None),
    ("PAD", "POD_DELIVERED", "B", None),
    ("PAD", "HOP_CMD", "B", None),
    ("DRONE", "LANDED", "B", None),
    ("PAD", "LATCH_CMD", "B", None),
    ("DRONE", "LATCH_REPORT", "B", None),
    ("PAD", "POD_LATCHED", "B", None),
    ("PAD", "SERVICE_COMPLETE", "B", "NORMAL"),
    ("DRONE", "MISSION_REQUEST", "B", "NORMAL"),
    ("FLEET_MANAGER", "MISSION_ASSIGN", None, "PARK"),
    ("DRONE", "DRONE_STATUS", "B", "PARKED"),
]
