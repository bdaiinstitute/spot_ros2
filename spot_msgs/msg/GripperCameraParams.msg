# Corresponds to https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#grippercameraparams

# See https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#grippercameraparams-cameramode
uint8 MODE_UNKNOWN = 0
uint8 MODE_640_480 = 11
uint8 MODE_640_480_120FPS_UYVY = 11
uint8 MODE_1280_720 = 1
uint8 MODE_1280_720_60FPS_UYVY = 1
uint8 MODE_1920_1080 = 14
uint8 MODE_1920_1080_60FPS_MJPG = 14
uint8 MODE_3840_2160 = 15
uint8 MODE_3840_2160_30FPS_MJPG = 15
uint8 MODE_4096_2160 = 17
uint8 MODE_4096_2160_30FPS_MJPG = 17
uint8 MODE_4208_3120 = 16
uint8 MODE_4208_3120_20FPS_MJPG = 16

# See https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#grippercameraparams-ledmode
uint8 LED_MODE_UNKNOWN = 0
uint8 LED_MODE_OFF = 1
uint8 LED_MODE_TORCH = 2

# See https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#hdrparameters
uint8 HDR_UNKNOWN = 0
uint8 HDR_OFF = 1
uint8 HDR_AUTO = 2
uint8 HDR_MANUAL_1 = 3
uint8 HDR_MANUAL_2 = 4
uint8 HDR_MANUAL_3 = 5
uint8 HDR_MANUAL_4 = 6

# See https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#roiparameters-roiwindowsize
uint8 ROI_WINDOW_SIZE_UNKNOWN = 0
uint8 ROI_WINDOW_SIZE_1 = 1
uint8 ROI_WINDOW_SIZE_2 = 2
uint8 ROI_WINDOW_SIZE_3 = 3
uint8 ROI_WINDOW_SIZE_4 = 4
uint8 ROI_WINDOW_SIZE_5 = 5
uint8 ROI_WINDOW_SIZE_6 = 6
uint8 ROI_WINDOW_SIZE_7 = 7
uint8 ROI_WINDOW_SIZE_8 = 8

uint8 camera_mode

float32 brightness
float32 contrast
float32 saturation
float32 gain

bool exposure_auto
float32 exposure_absolute
float32[2] exposure_roi_position
uint8 exposure_roi_size

bool focus_auto
float32 focus_absolute
float32[2] focus_roi_position
uint8 focus_roi_size
bool draw_focus_roi_rectangle

uint8 hdr

uint8 led_mode
float led_torch_brightness

bool white_balance_temperature_auto
float32 white_balance_temperature

float32 gamma
float32 sharpness
