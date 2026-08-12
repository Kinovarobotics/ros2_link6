# Base Cyclic Feedback

uint32 active_state         # Current arm state, see ARMSTATE_* constants below
string active_state_name    # Human-readable name of active_state
uint32 operating_mode       # Current operating mode, see OPERATING_MODE_* constants below
string operating_mode_name  # Human-readable name of operating_mode
bool enabling_device_state  # True if the physical enabling/deadman device is pressed

# --- Operating Mode Constants -------------------------------------
uint32 OPERATING_MODE_UNSPECIFIED    = 0
uint32 OPERATING_MODE_JOG_MANUAL     = 1
uint32 OPERATING_MODE_HAND_GUIDING   = 2
uint32 OPERATING_MODE_HOLD_TO_RUN    = 3
uint32 OPERATING_MODE_AUTO           = 4
uint32 OPERATING_MODE_MONITORED_STOP = 5

# --- Active State (ArmState) Constants -----------------------------
uint32 ARMSTATE_UNSPECIFIED                    = 0
uint32 ARMSTATE_BASE_INITIALIZATION            = 1
uint32 ARMSTATE_IDLE                           = 2
uint32 ARMSTATE_INITIALIZATION                 = 3
uint32 ARMSTATE_IN_FAULT                       = 4
uint32 ARMSTATE_MAINTENANCE                    = 5
uint32 ARMSTATE_SERVOING_LOW_LEVEL             = 6
uint32 ARMSTATE_SERVOING_READY                 = 7
uint32 ARMSTATE_SERVOING_PLAYING_SEQUENCE      = 8
uint32 ARMSTATE_SERVOING_MANUALLY_CONTROLLED   = 9
uint32 ARMSTATE_BRAKE_RELEASING                = 10
uint32 ARMSTATE_POSITION_UNKNOWN               = 11
uint32 ARMSTATE_ARM_OPERATIONAL                = 12
uint32 ARMSTATE_RECOVERY                       = 13
uint32 ARMSTATE_CONTROLLER_INITIALIZATION      = 14
uint32 ARMSTATE_WAIT_FOR_POSITION_CONFIRMATION = 15
uint32 ARMSTATE_IN_FAULT_POWERED_OFF           = 16
uint32 ARMSTATE_IN_UNRECOVERABLE_FAULT         = 17