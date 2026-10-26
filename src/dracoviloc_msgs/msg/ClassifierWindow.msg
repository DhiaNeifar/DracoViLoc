# One inference window of an audio drone classifier (MobileNetV2), published on
# /mobilenetv2/diagnostics. Diagnostic only: it never drives motion.

std_msgs/Header header          # window end time; frame_id = microphone frame

uint8 channel                   # /sss channel == /sst slot
int32 track_id                  # ODAS track id in that slot (0 = empty)
float32 activity                # ODAS track activity
float32 probability             # drone probability of this window
bool vote                       # probability >= threshold
uint8 votes                     # positive votes in the current window of votes
uint8 vote_window               # number of votes considered
bool decision                   # votes >= votes_required
bool actionable                 # the source was fresh and active (not diagnostic-only)
float32 inference_ms            # TensorRT inference time
