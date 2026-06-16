uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32 fz_des_body              # desired body-z specific force from final thrust_sp for FTC INDI allocation [m/s^2]
float32 specific_force_magnitude # raw acceleration-demand specific force magnitude for diagnostics [m/s^2]
float32 primary_axis_nz          # FTC primary-axis body z component used for thrust compensation
float32 thrust_effectiveness     # -primary_axis_nz constrained to the compensation range

float32[3] acceleration_sp       # position controller acceleration setpoint in NED [m/s^2]
float32[3] thrust_sp             # normalized thrust setpoint in NED, kept for comparison/logging

# TOPICS vehicle_ftc_physical_setpoint
