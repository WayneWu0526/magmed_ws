Header header              # ROS standard header

float32[3] nJOY1             # Three axes of the big joystick
float32[2] nJOY2             # Two axes of the first (left) small joystick
float32[2] nJOY3             # Two axes of the second (right) small joystick
bool bJOYD                 # Big joystick button
uint16 POTA                # Potentiometer A
uint16 POTB                # Potentiometer B
int32 BANA                 # Rotary switch A
int32 BANB                 # Rotary switch B
int16 ENCA                 # Encoder A
int16 ENCB                 # Encoder B
bool[5] TOG                # Toggle switches (5 in total)
bool[6] BUT                # Push buttons (6 in total)
