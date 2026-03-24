# TOPICS control_allocator_ftc_debug
uint32 MESSAGE_VERSION = 0

uint64 timestamp                        # time since system start (microseconds)
uint64 timestamp_sample                 # timestamp of the input setpoints used for this allocation cycle

bool ftc_active                        # true when FTC logic is active for this cycle
bool fault_trigger_active              # true when the configured FTC fault scenario has been triggered
bool degraded_allocation_active        # true when degraded 3x3 allocation is applied this cycle
bool output_fault_active               # true when output-stage fault clamping is applied this cycle
uint8 fault_motor_index                # 1-based motor index selected by FTC, 0 if inactive
uint16 handled_motor_failure_mask      # motors removed from effectiveness by failure handling
uint16 motor_stop_mask                 # motors stopped by failure injection

float32 loe                            # retained effectiveness ratio used by FTC
float32 nominal_fault_command          # nominal command before FTC limiting
float32 applied_fault_command          # command applied to the failed motor after FTC limiting
float32 fault_command_limit            # FTC limit for the failed motor command
float32 residual_yaw_moment            # yaw moment left unallocated after FTC reallocation

float32[3] torque_setpoint             # body torque setpoint used for the cycle
float32[3] thrust_setpoint             # body thrust setpoint used for the cycle

int8 ACTUATOR_SATURATION_OK        =  0
int8 ACTUATOR_SATURATION_UPPER_DYN =  1
int8 ACTUATOR_SATURATION_UPPER     =  2
int8 ACTUATOR_SATURATION_LOWER_DYN = -1
int8 ACTUATOR_SATURATION_LOWER     = -2

int8[12] actuator_saturation           # saturation state of each motor after final clip

float32[12] pre_ftc_control            # motor inputs after normal allocation, before FTC reallocation
float32[12] post_ftc_control           # motor inputs after FTC reallocation and stopped-motor handling
float32[12] final_control              # motor inputs after output-stage fault clamping and final publish shaping
