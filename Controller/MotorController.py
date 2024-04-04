import odrive
from odrive.enums import *
odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE