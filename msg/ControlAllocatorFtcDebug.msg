uint64 timestamp                        # time since system start (microseconds)
uint64 timestamp_sample                 # timestamp of the input setpoints used for this allocation cycle

bool ftc_active                        # true when FTC logic is active for this cycle
bool fault_trigger_active              # true when the configured FTC fault scenario has been triggered
bool indi_control_active               # true when the FTC INDI attitude/allocation controller is active
bool degraded_allocation_active        # legacy field; false because pseudo-inverse degraded allocation is not used
bool output_fault_active               # true when output-stage fault clamping is applied this cycle
uint8 fault_motor_index                # 1-based motor index selected by FTC, 0 if inactive
uint8 ftc_mode                         # 0=normal, 1=single INDI, 2=dual-opposing INDI, 3=fault nominal
uint16 fault_motor_mask                # motors commanded failed by the FTC scenario
uint16 remaining_motor_mask            # motors used by the active FTC allocator
uint16 handled_motor_failure_mask      # motors removed from effectiveness by failure handling
uint16 motor_stop_mask                 # motors stopped by failure injection

float32 loe                            # retained effectiveness ratio used by FTC
float32 nominal_fault_command          # nominal command before FTC limiting
float32 applied_fault_command          # command applied to the failed motor after FTC limiting
float32 fault_command_limit            # FTC limit for the failed motor command
float32 residual_yaw_moment            # yaw moment left unallocated after FTC reallocation
float32 chi                            # dual-opposing INDI output angle
float32 sl                             # dual-opposing INDI remaining-pair sign
float32 sn                             # dual-opposing INDI spin handedness sign
float32[2] y                           # dual-opposing INDI outputs/accelerations used for debugging
float32[2] nu                          # dual-opposing INDI pseudo-input
float32[2] indi_u                      # dual-opposing INDI remaining motor commands

uint8 INDI_FAIL_NONE                   = 0
uint8 INDI_FAIL_NOT_TRIGGERED          = 1
uint8 INDI_FAIL_MOTOR_COUNT            = 2
uint8 INDI_FAIL_FAULT_COLUMN           = 3
uint8 INDI_FAIL_HEALTHY_COUNT          = 4
uint8 INDI_FAIL_THRUST_EFFECTIVENESS   = 5
uint8 INDI_FAIL_ROTOR_COEFFICIENT      = 6
uint8 INDI_FAIL_LOW_THRUST             = 7
uint8 INDI_FAIL_SINGULAR_EFFECTIVENESS = 8

bool indi_success                      # true when the single-rotor INDI allocator rewrote healthy motor commands
uint8 indi_fail_reason                 # INDI_FAIL_* reason for inactive or failed single-rotor INDI allocation
int8 indi_fault_column                 # allocation matrix column for the failed motor, -1 if unavailable
uint8 indi_healthy_count               # number of healthy motor columns collected for single-rotor INDI
bool indi_matrix_invertible            # true when the single-rotor INDI 3x3 effectiveness matrix was invertible
float32 indi_collective_thrust         # positive collective thrust demand used to gate single-rotor INDI
float32 indi_effectiveness_det         # determinant of the single-rotor INDI 3x3 effectiveness matrix
float32[4] indi_eff_mx                 # raw PX4 roll moment effectiveness for motors 1-4
float32[4] indi_eff_my                 # raw PX4 pitch moment effectiveness for motors 1-4
float32[4] indi_eff_fz                 # raw PX4 vertical thrust effectiveness for motors 1-4
float32[4] indi_ratio_mx_abs_fz        # roll moment per absolute vertical thrust effectiveness
float32[4] indi_ratio_my_abs_fz        # pitch moment per absolute vertical thrust effectiveness
float32[9] indi_g_raw                  # row-major physical INDI Ghat before numerical scaling
float32[9] indi_g_scaled               # row-major physical INDI Ghat after row scaling
float32[3] indi_row_scale              # row scaling applied to Ghat and control error
float32 indi_cond_proxy                # simple conditioning proxy based on scaled row norms
float32 indi_scaled_det                # determinant of the row-scaled single-rotor INDI matrix
float32[3] indi_nu_in                  # physical INDI virtual input [rad/s^2, rad/s^2, m/s^2]
float32[3] indi_y_dot_f                # filtered physical INDI output derivative [rad/s^2, rad/s^2, m/s^2]
float32[3] indi_error                  # physical INDI error nu_in - y_dot_f
float32 indi_fz_des                    # physical body-z specific force command reconstructed from normalized thrust
float32 indi_fz_error_int              # integral of physical body-z specific-force error
float32[3] indi_delta_omega2           # healthy-motor omega^2 increment solved by INDI
float32[4] indi_omega2_f               # filtered motor omega^2 feedback for motors 1-4
float32[4] indi_omega2_cmd             # commanded motor omega^2 after INDI allocation for motors 1-4

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
float32[12] final_control              # motor inputs after slew-rate limiting and final clipping
