# One direction measurement processed by dracoviloc_ekf, published on
# /ekf/diagnostics for every measurement, accepted or not.

std_msgs/Header header          # measurement stamp; frame_id = tracking frame

string source                   # input topic, e.g. /yolo/direction
string decision                 # initialized | accepted | reacquired |
                                # rejected_nis | rejected_bearing_gate |
                                # rejected_first_arrival
float64 nis                     # normalised innovation squared (NaN if not computed)
float64 nis_gate                # threshold the NIS was compared against
float64 innovation_deg          # angle between measurement and predicted direction
float64 measurement_noise       # R used for this source
float64 processing_delay_s      # node receive time minus measurement stamp

geometry_msgs/Vector3 measurement   # unit vector
geometry_msgs/Vector3 estimate      # filter direction after this measurement
