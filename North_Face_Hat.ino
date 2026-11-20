/* Arduino Flora Wearable Project for Compass Hat with Haptic (Vibrating) Feedback When Facing North
 * Instructions: Switch on hat, then turn and tilt the hat in all directions to record the min/max magnetic field strengths. 
 * Hat will vibrate when you face north. 
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h> //for 9DOF sensor
#include <Adafruit_LSM9DS0.h> //for 9DOF sensor 
#include <LSM303_mod.h> //for 9DOF sensor - Modified library to account for direction of XYZ coordinates
#include <Adafruit_DRV2605.h> //for haptic controller 

// Initialize Variable for Buzz Count
int BuzzCount;

//For Compass Calibration
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
LSM303::vector<int16_t> running_min_converted = {32766, 32766, 32766}, running_max_converted = {-32767, -32767, -32767};
char report[80];


// Assign a unique base ID for this sensor  
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Configures the gain and integration time for the TSL2561
void configureSensor(void)
{
  
  // Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

}

//Declare  drv
Adafruit_DRV2605 drv;

//Plays one effect from the DRV2605 effect library, then waits pauseMs
void playEffect(uint8_t effect, int pauseMs)
{
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0); //end of waveforms
  drv.go();
  delay(pauseMs);
}

//Widens [lo, hi] to include v. Returns true only if it grew by more than tolerance.
bool widen(int16_t &lo, int16_t &hi, int16_t v, long tolerance)
{
  bool grewALot = false;
  if (v < lo) { grewALot = ((long)lo - v) > tolerance; lo = v; }
  if (v > hi) { grewALot = ((long)v - hi) > tolerance; hi = v; }
  return grewALot;
}


void setup(void) 
{
  delay(5000); // Delay 5 seconds for serial port to get ready

  Serial.begin(9600);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");

  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
// Select library for haptic controller 

  drv.begin();
  
  drv.selectLibrary(1);
  
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG); 

  //Initialize compass before calibration
  Wire.begin();
  compass.init();
  compass.enableDefault();

  //---- Calibration ----
  //Put the hat on, turn slowly through at least one full circle, and look up and down / tilt your head a few times.
  //You don't need to stop moving: calibration finishes on its own once ALL of these are true:
  //  1. X and Y have each swung through a similar, large range  -> you have faced every direction
  //  2. Z has swung through a reasonable share of that range     -> you have tilted your head enough
  //  3. none of the min/max values has grown for STABLE_TIME_MS  -> there is nothing new left to find
  //Standing still early does NOT end calibration, because 1 and 2 are not met until you have actually turned.
  //Signals: 1 click = calibration started, 3 buzzes = calibrated, 1 long buzz = gave up after MAX_CAL_TIME_MS (rough calibration).

  const unsigned long STABLE_TIME_MS  = 5000;   //min/max must stop growing for this long
  const unsigned long MAX_CAL_TIME_MS = 120000; //start anyway after 2 minutes
  const long  MIN_RANGE  = 1000;  //each of X and Y must swing at least this many raw counts (rejects sensor noise)
  const float XY_BALANCE = 0.75;  //smaller of the X/Y ranges must be at least 75% of the larger one
  const float Z_FRACTION = 0.40;  //Z range must be at least 40% of the X/Y range

  playEffect(1, 500); //one click: calibration has started

  compass.read();
  running_min = compass.m; //start min and max from a real reading
  running_max = compass.m;

  unsigned long calStart   = millis();
  unsigned long lastGrowth = millis();
  bool calibrated = false;
  int printCounter = 0;

  while (millis() - calStart < MAX_CAL_TIME_MS)
  {
    compass.read();

    //Only growth bigger than ~2% of an axis' range (at least 30 counts) counts as "still finding new values",
    //so sensor noise nudging a max up by a few counts doesn't keep calibration running forever
    long tolX = max(30L, ((long)running_max.x - running_min.x) / 50);
    long tolY = max(30L, ((long)running_max.y - running_min.y) / 50);
    long tolZ = max(30L, ((long)running_max.z - running_min.z) / 50);

    //Single | on purpose so all three axes are always updated
    bool grew = widen(running_min.x, running_max.x, compass.m.x, tolX)
              | widen(running_min.y, running_max.y, compass.m.y, tolY)
              | widen(running_min.z, running_max.z, compass.m.z, tolZ);
    if (grew) lastGrowth = millis();

    long rangeX = (long)running_max.x - running_min.x;
    long rangeY = (long)running_max.y - running_min.y;
    long rangeZ = (long)running_max.z - running_min.z;
    long xySmall = min(rangeX, rangeY);
    long xyBig   = max(rangeX, rangeY);

    bool xyGood = (xySmall >= MIN_RANGE) && (xySmall >= XY_BALANCE * xyBig);
    bool zGood  = (rangeZ >= Z_FRACTION * xyBig);
    bool stable = (millis() - lastGrowth >= STABLE_TIME_MS);

    if (++printCounter >= 10) //print progress about once a second
    {
      printCounter = 0;
      snprintf(report, sizeof(report), "range X:%6ld Y:%6ld Z:%6ld  turned:%s tilted:%s settled:%s",
        rangeX, rangeY, rangeZ, xyGood ? "yes" : "no", zGood ? "yes" : "no", stable ? "yes" : "no");
      Serial.println(report);
    }

    if (xyGood && zGood && stable)
    {
      calibrated = true;
      break;
    }

    delay(100);
  }


  //Setting the min/max values to calibrate the compass
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){running_min.x, running_min.y, running_min.z};
  compass.m_max = (LSM303::vector<int16_t>){running_max.x, running_max.y, running_max.z};

  //Tell the wearer calibration is done
  if (calibrated)
  {
    Serial.println("Calibrated!\n");
    for (int i = 0; i < 3; i++) playEffect(14, 400); //three strong buzzes
  }
  else
  {
    Serial.println("Calibration timed out - using what was collected\n");
    playEffect(16, 1500); //one long buzz
  }
  delay(1500); //short gap so the "done" signal isn't confused with a north buzz

}


void loop(void) 

{  

  compass.read();
  float heading = compass.heading();
  Serial.println(heading);  
  delay(100);
  
  //Buzz when entering a narrow window around north (350-10 degrees),
  //but only re-arm once the wearer has turned well away (more than 30 degrees off north).
  //The gap between the two windows stops sensor jitter at the edge from triggering repeat buzzes.
  bool facingNorth = (heading >= 350 || heading <= 10);
  bool wellAway    = (heading > 30 && heading < 330);

if (facingNorth)
  {
  if (BuzzCount < 1)
    {
    // buzz vibrating motor 
    drv.setWaveform(0, 110);  // Transition ramp up short smooth, 0% to 50%, see datasheet part 11.2
    drv.setWaveform(1, 3);  // strong click 30%, see datasheet part 11.2
    drv.setWaveform(2, 98);  // Transition ramp down short smooth, 50% to 0%, see datasheet part 11.2
    drv.setWaveform(3, 0);  // end of waveforms
    drv.go();
    BuzzCount++;
    delay(2000); // Wait after buzz
    Serial.println("Facing North - Vibrating!\n");

    }
  }
else if (wellAway)
  {
  BuzzCount = 0;
  }

  //Light up red on-board LED to show that hat is calibrated and working
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH); // turn the LED on (HIGH is the voltage level)
  
}
