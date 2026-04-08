#include "lib/packet.h"
#include "lib/sensor.h"
#include "lib/task.h"
#include "lib/stage.h"
#include "lib/queue.h"

#include "sensor/altemeter.h"
#include "sensor/gps.h"
#include "sensor/imu.h"
#include "sensor/Xbee.h"
#include "sensor/motor.h"
#include "sensor/logger.h"

#include "task/carnard.h"
#include "task/updatesensor.h"
#include "task/parachute.h"
#include "task/recieve.h"
#include "task/transmit.h"
#include "task/stagecheck.h"

#include <Arduino.h>