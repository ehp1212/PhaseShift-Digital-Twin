# ==============================
# PhaseShift Digital Twin
# Authoritative System Contract
# ==============================

builtin_interfaces/Time stamp

# -------- High-Level System Phase --------
uint8 phase

uint8 PHASE_BOOT=0
uint8 PHASE_CONNECTING=1
uint8 PHASE_SLAM_ACTIVE=2
uint8 PHASE_MAP_SAVED=3
uint8 PHASE_NAV_READY=4
uint8 PHASE_NAVIGATING=5
uint8 PHASE_ERROR=6

# -------- Error Handling --------
bool has_error
string error_message