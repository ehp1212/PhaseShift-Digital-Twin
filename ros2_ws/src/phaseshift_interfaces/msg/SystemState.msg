# ==============================
# PhaseShift Digital Twin
# Authoritative System Contract
# ==============================

builtin_interfaces/Time stamp

# -------- High-Level System Phase --------
uint8 phase

uint8 PHASE_BOOT=0
uint8 PHASE_SYSTEM_INITIALIZING=1

uint8 PHASE_SLAM_PREPARING=2
uint8 PHASE_SLAM_ACTIVE=3
uint8 PHASE_MAP_SAVING=4
uint8 PHASE_MAP_SAVED=5

uint8 PHASE_NAV_PREPARING=6
uint8 PHASE_NAV_READY=7
uint8 PHASE_NAV_EXECUTING=8

uint8 PHASE_ERROR=9

# -------- Error Handling --------
bool has_error
string error_message