/*
 * tank-bot-mk1.ino: controller for a simple obstacle-avoiding vehicle
 *
 * Copyright (C) 2020 Dan Crank (danno@danno.org)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * the vehicle is currently based on:
 *     Adafruit Circuit Playground Bluefruit
 *         https://www.adafruit.com/product/4333
 *     Adafruit Crickit
 *         https://www.adafruit.com/product/3093
 *     RoboPeak RPLIDAR
 *         https://www.adafruit.com/product/4010
 *     DFRobot Devastator tank chassis
 *         https://www.dfrobot.com/product-1477.html
 *         (this is the kit version that uses the 6V metal geared motors:
 *         https://www.dfrobot.com/product-1476.html)
 *
 * the Crickit is connected at its standard attachment points (see
 * https://learn.adafruit.com/adafruit-crickit-creative-robotic-interactive-construction-kit)
 *
 * use of RPLIDAR requires the non-standard library https://github.com/robopeak/rplidar_arduino
 * 
 * IMPORTANT NOTE: at the time of this writing, the RPLidar Adruino library has a bug. The
 * RPLidar::begin method is declared to return bool but it does not actually return a value.
 * In my testing, this caused the controller to lock up. This can be fixed by editing the
 * files RPLidar.h and RPLidar.cpp and changing the RPLidar::begin method to return void in
 * both places.
 * 
 *     void begin(HardwareSerial &serialobj);
 *
 * the RPLIDAR is connected as generally described in the documentation (see
 * http://www.robopeak.net/data/doc/rplidar/appnote/RPLDAPPN01-rplidar_appnote_arduinolib-enUS.pdf)
 *
 *     Pin              RPLIDAR                Circuit Playground
 *     1                GND                    GND (any)
 *     2                RX                     TX
 *     3                TX                     RX
 *     4                V5.0                   VOUT
 *     5                GND                    GND (any)
 *     6                MOTOCTL                AUDIO*
 *     7                VMOTO                  VOUT
 *
 *     * NOTE: the circuit playground AUDIO pin is also passed through to
 *     the speaker terminal on the crickit. do not connect anything to the
 *     speaker terminal or it will interfere with RPLIDAR motor control.
 */

#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_Crickit.h>
#include <Adafruit_seesaw.h>
#include <seesaw_motor.h>
#include <RPLidar.h>

/*
 * globals
 */

RPLidar lidar;
const unsigned int RPLIDAR_MOTOR = 12;

// keep a global buffer representing the most recent scan of the surroundings
// each bin is a ten-degree segment (0-9.99, 10-19.99, etc.)
// if you want to use a different bin / buffer size, keep in mind that the
// code is currently optimized for an even number of bins and an even
// number of degrees in each bin. so good values of NAV_BUFFER_SIZE would
// be 18, 20, 30, 36, 60, 90, 180
struct nav_scan
{
  unsigned int distanceNear;
  unsigned int pointCount;
};
const unsigned int NAV_BUFFER_SIZE = 36;
nav_scan navBuffer[NAV_BUFFER_SIZE];

void initNavBuffer()
{
  debug("InitNavBuffer()");
  for (unsigned int i = 0; i < NAV_BUFFER_SIZE; i++)
  {
    navBuffer[i].distanceNear = 0;
    navBuffer[i].pointCount = 0;
  }
}

Adafruit_Crickit crickit;
seesaw_Motor motorLeft(&crickit);
seesaw_Motor motorRight(&crickit);
bool crickitStarted = false;

/*
 * status codes
 */
const unsigned int STATUS_INITIALIZING = 0;
const unsigned int STATUS_LIDAR_CONNECTING = 1;
const unsigned int STATUS_SCANNING = 2;
const unsigned int STATUS_MANEUVERING = 3;
const unsigned int STATUS_DRIVING = 4;
const unsigned int STATUS_HALT_TEMPORARY = 5;
const unsigned int STATUS_HALT_PERMANENT = 6;

unsigned int vehicleStatus = STATUS_INITIALIZING;

/*
 * failure codes (if these go higher than 9, status() will
 * need to be changed to display the code in binary; currently
 * it just lights the single neopixel for the code)
 */
const unsigned int FAIL_DEBUG = 0;                // SETUP_ONLY set, so we halt after setup()
const unsigned int FAIL_CRICKIT = 1;              // crickit.begin() failed - check power

/*
 * configuration values
 */

// uncomment this line for debugging over a tethered USB connection
// #define USB_DEBUG

// set to TRUE to halt after setup, for testing only
const bool SETUP_ONLY = false;

// threshold for calling a halt based on unsafe vehicle attitude (0.0 - 10.0)
// lower values will allow a steeper tilt before halting.
const float TILT_THRESHOLD = 7.0;

// brightness of the circuitplayground's built-in neopixel ring (used for
// indicating vehicle status); scale is 0 - 255 but it gets BRIGHT.
const unsigned int PIXEL_BRIGHTNESS = 16;

// maximum motor throttles (0.0 - 1.0). with the current setup this needs
// to be about 0.8 or less to avoid brownouts (even with the capacitor).
const float MOTOR_FORWARD = 0.8;
const float MOTOR_REVERSE = -0.8;

// size of the frontal cone to check for obstacles, specified in number of
// sectors off-center. for example, if this parameter is set to 2, then the
// sector immediately ahead of the vehicle and the 2 sectors on either side
// will be watched. to continue the example, if NAV_BUFFER_SIZE=36, each
// sector is 10 degrees, so the area watched ends up being 50 degrees (5*10).
const unsigned int NAV_AVOID_CONE = 2;

// distance (in mm) from an obstacle at which the vehicle will maneuver to
// avoid the obstacle
const unsigned int NAV_AVOID_DISTANCE = 500;

// distance (in mm) of free space that must exist on a potential escape route
// for the vehicle to consider that route.
const unsigned int NAV_ESCAPE_DISTANCE = 1000;

// setting to collect more than one complete LIDAR rotation in a "scan".
// this must be at least 1. values higher than 1 may help the vehicle make
// better maneuvering decisions, but interfere with the actual maneuvering
// because the time spent scanning extends the length of the decision-making
// cycle.
const unsigned int OVERSCAN = 1;

// LIDAR motor speed (0-255)
const unsigned int LIDAR_MOTOR_SPEED = 255;

// minimum distance, inside which LIDAR data points are thrown out. this is
// used to filter out spurious data points at unreasonably close distances
// (inside the perimeter of the vehicle or thereabouts). value in mm.
// the datasheet lists minimum measurable distance as 150mm, so that's a
// good initial value.
const unsigned int LIDAR_MINIMUM_DISTANCE = 150;

// minimum quality, beneath which LIDAR data points are thrown out. this is
// a score assigned by the device, and its meaning is not documented (it is
// an unsigned byte coming from the device). in practice, most data points
// are observed to be quality = 15.
const unsigned int LIDAR_MINIMUM_QUALITY = 12;

/*
 * vehicle status functions
 */

const unsigned int NUM_PIXELS = 10;
const uint32_t WHITE = 0x00FFFFFF;
const uint32_t RED = 0x00FF0000;
const uint32_t GREEN = 0x0000FF00;
const uint32_t BLUE = 0x000000FF;
const uint32_t YELLOW = 0x00FF8800;
const uint32_t CYAN = 0x0000FFFF;
const uint32_t PURPLE = 0x008800FF;
const uint32_t BLACK = 0x00000000;

void setStatus(unsigned int newStatus, int failCode = -1)
{
  // set a status indicator of the appropriate color
  // each time status() is called, the lit pixel will move one position
  // clockwise around the ring, as an indicator that the control loop is
  // still running. if status is HALT_PERMANENT, the entire ring will
  // light red.
  static unsigned int statusPosition = 0;

  if (newStatus == vehicleStatus) return;
  vehicleStatus = newStatus;
  if (vehicleStatus == STATUS_HALT_PERMANENT)
  {
    debug("status: STATUS_HALT_PERMANENT");
    // for a permanent halt, light the ring up red
    // and set one position white as a fault code
    for (unsigned int i = 0; i < NUM_PIXELS; i++)
      if (i == failCode) CircuitPlayground.setPixelColor(i, WHITE);
      else CircuitPlayground.setPixelColor(i, RED);
    fullStop();
    // main control loop will recognize the halt status
  } else {
    CircuitPlayground.clearPixels();
    uint32_t color;
    switch (vehicleStatus)
    {
      case STATUS_DRIVING: color = GREEN; debug("status: STATUS_DRIVING"); break;
      case STATUS_HALT_TEMPORARY: color = YELLOW; debug("status: STATUS_HALT_TEMPORARY"); break;
      case STATUS_INITIALIZING: color = WHITE; debug("status: STATUS_INITIALIZING"); break;
      case STATUS_LIDAR_CONNECTING: color = PURPLE; debug("status: STATUS_LIDAR_CONNECTING"); break;
      case STATUS_SCANNING: color = CYAN; debug("status: STATUS_SCANNING"); break;
      case STATUS_MANEUVERING: color = BLUE; debug("status: STATUS_MANEUVERING"); break;
    }
    CircuitPlayground.setPixelColor(statusPosition, color);
    statusPosition = (statusPosition + 1) % NUM_PIXELS;
  }
}

void neopixelInit()
{
  debug("neopixelInit()");
  CircuitPlayground.clearPixels();
  // initialization display
  for (unsigned int i = 0; i < NUM_PIXELS; i++)
  {
    CircuitPlayground.setPixelColor(i, GREEN);
    delay(250);
  }
  for (unsigned int i = 0; i < NUM_PIXELS; i++)
  {
    CircuitPlayground.setPixelColor(i, BLACK);
    delay(250);
  }
  setStatus(STATUS_INITIALIZING);
}

// shut down all moving parts
void fullStop()
{
  debug("fullStop()");
  // shut down LIDAR motor
  analogWrite(RPLIDAR_MOTOR, 0);
  // shut down drive motors
  driveStop();
}

// shut down but go into a check loop to wait for soft-reset
// caller should return to main loop after calling this function
void haelp()
{
  if (vehicleStatus != STATUS_HALT_PERMANENT)
  {
    setStatus(STATUS_HALT_TEMPORARY);
    driveStop();
    // wait for a start-button press to return to main loop
    while (true)
    {
      if (CircuitPlayground.leftButton()) return;
      delay(200);
    }
  }
}

/*
 * drive motor functions
 */
void driveInit()
{
  debug("driveInit()");
  if (!crickit.begin())
  {
    setStatus(STATUS_HALT_PERMANENT, FAIL_CRICKIT);
  } else {
    crickitStarted = true;
    motorLeft.attach(CRICKIT_MOTOR_A1, CRICKIT_MOTOR_A2);
    motorRight.attach(CRICKIT_MOTOR_B1, CRICKIT_MOTOR_B2);
    setStatus(STATUS_INITIALIZING);
  }
}

void driveForward()
{
  debug("driveForward()");
  if (crickitStarted)
  {
    motorLeft.throttle(MOTOR_FORWARD);
    motorRight.throttle(MOTOR_FORWARD);
  }
}

void driveReverse()
{
  debug("driveReverse()");
  if (crickitStarted)
  {
    motorLeft.throttle(MOTOR_REVERSE);
    motorRight.throttle(MOTOR_REVERSE);
  }
}

void driveRotateRight()
{
  debug("driveRotateRight()");
  if (crickitStarted)
  {
    motorLeft.throttle(MOTOR_FORWARD);
    motorRight.throttle(MOTOR_REVERSE);
  }
}

void driveRotateLeft()
{
  debug("driveRotateLeft()");
  if (crickitStarted)
  {
    motorLeft.throttle(MOTOR_REVERSE);
    motorRight.throttle(MOTOR_FORWARD);
  }
}

void driveStop()
{
  debug("driveStop()");
  if (crickitStarted)
  {
    motorLeft.throttle(0.0);
    motorRight.throttle(0.0);
  }
}

/*
 * LIDAR functions
 */

// note: lidarInit only sets up the serial port and motor control.
// it does not actually attempt any communication with the device.
void lidarInit()
{
  debug("lidarInit()");
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 0);
  setStatus(STATUS_INITIALIZING);
}

// handshake with the lidar device and start scanning.
// return TRUE if connection was successful, FALSE otherwise.
bool lidarConnect()
{
  debug("lidarConnect()");
  setStatus(STATUS_LIDAR_CONNECTING);
  // try to detect RPLIDAR
  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info)))
  {
    debug("lidar.getDeviceInfo OK model=" + String(info.model) + " firmware_version=" + String(info.firmware_version) + " hardware_version=" + String(info.hardware_version));
    if (IS_OK(lidar.startScan()))
    {
      debug("lidar.startScan OK, starting motor");
      // start motor rotating
      analogWrite(RPLIDAR_MOTOR, LIDAR_MOTOR_SPEED);
      delay(1000);
      return true;
    }
  }
  debug("lidarConnect() FAILED!");
  return false;
}

// take one (or more) complete scans of the surroundings and update the buffer
// return TRUE if the buffer is updated, FALSE if there was a problem
// and we have to stop to re-connect to the LIDAR.
bool lidarScan()
{
  unsigned long startTime = millis();
  debug("lidarScan()");
  //derive a couple of unchanging values, for a bit of efficiency
  static unsigned int binSize = 360 / NAV_BUFFER_SIZE;
  static unsigned int halfABin = binSize / 2;
  initNavBuffer();
  unsigned int pointsCollected = 0;
  unsigned int pointsThrownOut = 0;
  unsigned int startBits = 0;
  setStatus(STATUS_SCANNING);
  while (true)
  {
    // wait for a data point
    if (IS_OK(lidar.waitPoint()))
    {
      RPLidarMeasurement point = lidar.getCurrentPoint();
      // we want this function to be fast - therefore round off the floating point values first thing
      unsigned int theAngle = round(point.angle);
      unsigned int theDistance = round(point.distance);
      if (point.startBit)
      {
        startBits++;
        if (startBits > OVERSCAN)
        {
          //finish the scan
          debug("Scan complete; " + String(pointsCollected) + " points collected, " + String(pointsThrownOut) + " thrown out");
          debug("Scan took " + String(millis() - startTime) + " milliseconds at overscan=" + OVERSCAN);
          return true;
        }
      }
      if ((point.quality >= LIDAR_MINIMUM_QUALITY) && (theDistance > LIDAR_MINIMUM_DISTANCE)) // filter out bad data points
      {
        pointsCollected++;
        // normalize the heading
        // with the current mounting, "forward" is the direction reported by the lidar as
        // 90 degrees. we need that to be stored as 180 degrees (putting that in the middle
        // of the range makes the buffer indexing simpler).
        theAngle += 90;
        if (theAngle >= 360) theAngle -= 360;
        // consolidating the awkwardness here: to make navigation somewhat better, we want
        // "straight ahead" to be in the center of a bin, not on the dividing line between
        // two bins. So navBuffer[0] is going to be directly aft and
        // navBuffer[NAV_BUFFER_SIZE/2] will be directly forward.
        unsigned int bin = (unsigned int)((theAngle + halfABin) / binSize);
        if (bin == NAV_BUFFER_SIZE) bin = 0;  // awkward wrap around conversion
        // save the nearest blip in each bin for obstacle detection
        if ((navBuffer[bin].pointCount == 0) || (point.distance < navBuffer[bin].distanceNear))
          navBuffer[bin].distanceNear = point.distance;
        navBuffer[bin].pointCount++;
      } else pointsThrownOut++;
    } else {
      // here if there's an error; return FALSE so the control loop
      // can stop the vehicle and recover
      debug("lidarScan() timed out while waiting for data point");
      return false;
    }
  }
}

// old demo code that displayed the heading to the nearest object as a red light
// on the circuitplayground neopixel ring - no longer used, but keeping it as an example
//void lidarDemo()
//{
//  if (IS_OK(lidar.waitPoint()))
//  {
//    status(STATUS_OK);
//    float distance = lidar.getCurrentPoint().distance; //distance value in mm
//    float angle    = lidar.getCurrentPoint().angle; //angle value in degrees
//    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point belongs to a new scan
//    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
//
//    //test! find the nearest object in the current scan and mark it with a red neopixel
//    //don't know yet what angle value corresponds to what vehicle heading
//    if (startBit)
//    {
//      // normalize the heading - 90 degrees is forward
//      theAngle -= 90.0;
//      if (theAngle < 0.0) theAngle += 360.0;
//      // figure out which pixel corresponds to this angle
//      // pixels are numbered counterclockwise 0-9
//      // with current mounting, a 0 degree heading is between 4 and 5
//      int iDirIndex = 0;
//      iDirIndex = (int)(4.5 - (theAngle / 36.0));
//      if (iDirIndex < 0) iDirIndex += 10;
//      CircuitPlayground.clearPixels();
//      CircuitPlayground.setPixelColor(iDirIndex, RED);
//      //Serial.println("Angle=" + String(theAngle) + " Distance=" + String(theDistance) + " Quality=" + String(theQuality) + " Points=" + String(iPointCount) + " Index=" + String(iDirIndex));
//      theAngle = 0.0;
//      theDistance = 10000.0;
//      theQuality = 0;
//      iPointCount = 0;
//    }
//    // quality score appears to be a max of 15, but seems to get enough 15s
//    // that we can throw everything else out. we also seem to get a lot of
//    // spurious data points at 0 distance - filter them out too
//    if ((quality >= 15) && (distance > 0.0))
//    {
//      if (distance < theDistance)
//      {
//        theDistance = distance;
//        theAngle = angle;
//        theQuality = quality;
//      }
//      iPointCount++;
//    }
//  } else {
//    lidarConnect();
//  }  
//}

/*
 * navigation / obstacle-avoidance functions
 */

// return the bin index of the nearest obstruction, or -1 if no obstructions are visible
// inside the NAV_AVOID_CONE and within NAV_AVOID_DISTANCE millimeters.
int navFindObstruction(int sector = (NAV_BUFFER_SIZE / 2), int distance = NAV_AVOID_DISTANCE)
{
  debug("navFindObstruction()");
  int bin = -1;
  unsigned int range = distance + 1;
  for (int i = sector - NAV_AVOID_CONE; i <= sector + NAV_AVOID_CONE; i++)
  {
    int realIndex = (i + NAV_BUFFER_SIZE) % NAV_BUFFER_SIZE;  // handle indexes that span buffer boundaries
    if (navBuffer[realIndex].pointCount == 0)
    {
      debug("I have no data for sector " + String(i) + ", declaring that sector obstructed.");
      //bin = realIndex;
      //range = 0;
    } else if ((navBuffer[realIndex].distanceNear < distance) &&
             (navBuffer[realIndex].distanceNear < range)) {
      bin = realIndex;
      range = navBuffer[realIndex].distanceNear;
    }
  }
  if (bin == -1)
    debug("No obstruction detected");
  else
    debug("Nearest obstruction detected in sector " + String(bin) + ", range=" + String(range) + "mm");
  return bin;
}

// look at the entire nav buffer (360 degrees) and find the best escape route
// from the given obstruction, or -1 if no escape can be found
int navFindEscape()
{
  debug("navFindEscape()");
  int bin = -1;
  int bestScore = NAV_BUFFER_SIZE / 2;
  for (int i = 0; i < NAV_BUFFER_SIZE; i++)
  {
    // for each bin, score it as a possible escape.
    // determine the number of sectors we would have to turn to reach this heading
    int deviation = (NAV_BUFFER_SIZE / 2) - i;  // halfway point of the buffer is straight ahead
    deviation = abs(deviation);  // arduino reference says not to nest operations inside abs()
    // check to see if the current buffer would read as unobstructed if the vehicle were pointed
    // to the heading being evaluated. if yes, the route gets a score by its deviation (lower
    // being better) and we keep the lowest score.
    if ((navFindObstruction(i, NAV_ESCAPE_DISTANCE) == -1) && (deviation < bestScore))
    {
      debug("New high score: sector " + String(i) + " deviation=" + String(deviation));
      bin = i;
      bestScore = deviation;
    }
  }
  if (bin == -1) debug("No escape found.");
  return bin;
}

// based on the current nav buffer, rotate to the best escape heading.
// return true if the vehicle is now pointed in a good direction,
// false if not...except that losing the lidar connection will still
// return true, because we don't want to TEMPORARY_HALT in that case.
bool navEscape()
{
  debug("navEscape()");
  driveStop();
  // find the escape route
  int escape = navFindEscape();
  if (escape == -1)
  {
    // oh noes, no escape can be found
    debug("No escape found, navEscape returning FALSE");
    return false;
  } else if (escape == (NAV_BUFFER_SIZE / 2)) {
    // special case: navFindEscape told us to go the direction we were already going...
    // since navFindEscape isn't supposed to refresh the buffer, this probably means
    // something is confused
    debug("navFindEscape returned sector " + String(escape) + ", current heading? navEscape returning TRUE");
    return true;
  } else {
    // figure out if we're turning left or right, then start turning
    if (escape < (NAV_BUFFER_SIZE / 2)) driveRotateLeft();
    else driveRotateRight();
    while (true)
    {
      if (!situationCheck()) return false;
      // re-scan
      if (!lidarScan())
      {
        // scan error
        // return true here so main loop can try to reconnect and recover
        debug("LIDAR timeout - navEscape returning TRUE to attempt reconnect");
        return true;
      }
      // check for clear road ahead
      if (navFindObstruction() == -1) return true;  // get in losers
      // otherwise keep looking
    }
  }
}

/*
 * situation check functions
 */

// return TRUE if the vehicle is in an acceptable situation to proceed,
// FALSE if not
bool situationCheck()
{
  // check to see if the vehicle is at an excessive tilt angle
  // simplest way seems to be to check if the Z vector (down)
  // is less than a threshold
  float currentTilt = CircuitPlayground.motionZ();
  if (currentTilt <= TILT_THRESHOLD)
  {
    debug("currentTilt=" + String(currentTilt) + ", calling for a halt");
    return false;
  }
  // check to see if stop button is being pressed
  if (CircuitPlayground.rightButton())
  {
    debug("Manual halt request (stop button pushed)");
    return false;
  }
  return true;
}

/*
 * debug functions
 */
void debug(String str)
{
#ifdef USB_DEBUG
  String t = "00000000" + String(millis());
  t.remove(0, t.length() - 8);
  Serial.println(t + " | " + str);
  Serial.flush();
#endif
}

void setup()
{
  CircuitPlayground.begin(PIXEL_BRIGHTNESS);
#ifdef USB_DEBUG
  Serial.begin(57600);
  debug("setup()");
#endif
  // all other setup goes below here
  // neopixelInit should remain first
  // each init function should end with a call to status(STATUS_INITIALIZING)
  // so the counter will advance; if we freeze during init we'll see how many
  // stages we got through
  if (vehicleStatus != STATUS_HALT_PERMANENT) neopixelInit();
  if (vehicleStatus != STATUS_HALT_PERMANENT) driveInit();
  if (vehicleStatus != STATUS_HALT_PERMANENT) lidarInit();
  if (SETUP_ONLY && (vehicleStatus != STATUS_HALT_PERMANENT))
    setStatus(STATUS_HALT_PERMANENT, FAIL_DEBUG);
  // start up in TEMPORARY_HALT mode...wait for button push to begin
  haelp();
}

unsigned long loopClock = millis();

void loop()
{
  // MAIN CONTROL LOOP
  unsigned long newClock = millis();
  debug("loop() [last loop took " + String(newClock - loopClock) + "ms]");
  loopClock = newClock;
  // check to see if vehicle has halted
  if (vehicleStatus == STATUS_HALT_PERMANENT)
  {
    delay(2000);
    return;
  }
  // check to see if vehicle has capsized, been picked up, etc.
  if (!situationCheck())
  {
    haelp();
    return;
  }

  // scan the surroundings
  debug("***MAIN CONTROL LOOP***");
  if (lidarScan())
  {
    int obstruction = navFindObstruction();
    if (obstruction == -1)
    {
      debug("No obstacles ahead - driving forward.");
      // no obstacles ahead, start moving forward
      setStatus(STATUS_DRIVING);
      driveForward();
    } else {
      debug("Obstacle detected in sector " + String(obstruction) + ", stopping.");
      driveStop();
      // turn to a new heading
      setStatus(STATUS_MANEUVERING);
      bool ok = navEscape();
      if (!ok)
        // here if there was a problem during maneuvering
        // so wait to be rescued
        haelp();
    }
  } else {
    // LIDAR error...stop and try to reconnect
    debug("LIDAR error (main loop)...attempting to reconnect");
    driveStop();
    lidarConnect();
  }
}
