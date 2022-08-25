#include "Arduino.h"
#include <FastLED.h>
#include <FastLEDPainter.h>
#include <EEPROM.h>
#include <CircularBuffer.h>

/*
 * Utilizing FastLED painter libary and some animations from Damian Schneider.
 */

/*
 * LED animation members, constants, headers
 */
const int duration = 1000; // number of loops to run each animation for

#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif

// Fixed definitions cannot change on the fly.
#define LED_DT 4        // Data pin to connect to the strip. For APA strips this generally the GREEN wire.
#define LED_CK 5        // For APA strips this generally the YELLOW wire.
#define COLOR_ORDER BGR // Are they RGB, GRB or what??
#define LED_TYPE APA102 // Don't forget to change LEDS.addLeds
#define NUM_LEDS 288    // 144                                         //Number of LEDs on the strip

// Initialize changeable global variables.
uint8_t max_bright = 128; // Overall brightness definition. It can be changed on the fly.

struct CRGB leds[NUM_LEDS];

// create one canvas and one brush with global scope
FastLEDPainterCanvas pixelcanvas = FastLEDPainterCanvas(NUM_LEDS);  // create canvas, linked to the FastLED library (canvas must be created before the brush)
FastLEDPainterBrush pixelbrush = FastLEDPainterBrush(&pixelcanvas); // crete brush, linked to the canvas to paint to

// create an additional "mirrored" brush, painting on the same canvas as the globally defined brush which ought to mirror the original for use with 288 leds in the dripReadMirror animation
FastLEDPainterBrush pixelbrushM = FastLEDPainterBrush(&pixelcanvas); // crete brush, linked to the canvas to paint to

// FOR BOUNCY BALLS ANIMATION
// create additional brushes, painting on the same canvas as the globally defined brush
FastLEDPainterBrush pixelbrushBB2 = FastLEDPainterBrush(&pixelcanvas); // crete brush, linked to the canvas to paint to
FastLEDPainterBrush pixelbrushBB3 = FastLEDPainterBrush(&pixelcanvas); // crete brush, linked to the canvas to paint to

/*
 * I2C members, constants and headers
 */
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#include <NXPMotionSense.h>
NXPMotionSense imu;
NXPSensorFusion filter;

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// default I2C address is 0x68
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
const bool DEBUG_OUTPUT = true;
const bool YAW_ANIMATION_CONTROL = false;
const uint8_t DIAL_SENSITIVITY = 35;
const uint8_t ADDRESS_ANIMATION_NUM = 0;
const uint8_t ANIMATION_COUNT = 4;

bool blinkState = false;
uint8_t YAW = 2;
uint8_t ROLL = 1;
uint8_t speedController = ROLL;
uint8_t secondaryController = YAW;
bool dynamiclyAssignMotionControls = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion qc;       // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// CircularBuffer<int, 100> rollHistory;
unsigned long time = 0;
float roll_rate = 0.0;     // roll rate
float old_roll_rate = 0.0; // roll rate
const float ROLL_THRESHOLD = 15.0;
bool spinning_animation = false;

uint8_t randomShow;
uint8_t animationNumber;
uint16_t programNotch;
static signed int speed = 1;
static signed int lastSpeed = speed;
byte eprom;

// static unsigned int hue = 0; //color hue to set to brush
CHSV brushcolor; // HSV color definition

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
        mpuInterrupt = true;
}

void setup()
{
        delay(1000); // sanity check delay

        // put your setup code here, to run once:
        // LEDS.addLeds<LED_TYPE, LED_DT, LED_CK, COLOR_ORDER>(leds, NUM_LEDS); // Use this for WS2801 or APA102

        FastLED.addLeds<LED_TYPE, COLOR_ORDER>(leds, NUM_LEDS);
        pinMode(7, OUTPUT);
        digitalWrite(7, HIGH);

        // setupI2C();

        // initI2C();
        // readMpu();
        time = millis();

        setupPropShield();
        readPropShield();

        setupLedAnimations();
        randomShow = random(10);
        Serial.print("random number is:");
        Serial.print(randomShow);
        Serial.print("\n");

        // reading EEPROM persisted animation number
        animationNumber = EEPROM.read(ADDRESS_ANIMATION_NUM);
        Serial.print("ANIMATION EEPROM :");
        Serial.print(animationNumber);
        Serial.print("\n");

        if (!YAW_ANIMATION_CONTROL)
                saveAnimationNum((animationNumber + 1) % ANIMATION_COUNT);

        if (DEBUG_OUTPUT)
                delay(200); // delay so serial interface on teensy can be created (it's a virtual interface)

        if (dynamiclyAssignMotionControls)
                initDynamicControlsDuringBatteryCheck();
        else
                batteryCheck();
}

// Animation
void blink(float voltage)
{
        int blinkCount = 0;
        if (voltage > 4.2)
                blinkCount = 5;
        else if (voltage > 4.0)
                blinkCount = 4;
        else if (voltage > 3.8)
                blinkCount = 3;
        else if (voltage > 3.6)
                blinkCount = 2;
        else if (voltage > 3.4)
                blinkCount = 1;

        for (int i = 0; i < blinkCount; i++)
        {
                pixelbrush.setFadeSpeed(90);
                pixelbrush.setFadein(false); // brightness will fade-in if set to true
                pixelbrush.setFadeout(true);
                pixelbrush.setBounce(false);

                pixelbrush.setSpeed(0); // brush moving speed

                brushcolor.s = 255; // full saturation
                int brightness = 255;
                brushcolor.v = brightness; // random (peak) brighness

                pixelbrush.setColor(brushcolor); // set new color to the bursh

                FastLED.clear();
                FastLED.show();
                delay(200);

                pixelbrush.paint();     // paint the brush to the canvas (and update the brush, i.e. move it a little)
                pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

                FastLED.show();
                delay(200);
        }
}

void batteryCheck()
{
        int sensorValue = analogRead(A0);                   // read the A0 pin value
        float voltage = sensorValue * (5.00 / 1600.00) * 2; // convert the value to a true voltage.
        Serial.print("voltage = ");
        Serial.print(voltage);
        Serial.println(" V");

        blink(voltage); // blinks indicating approximate battery level
}

// Allows one to orient the stave in position one would like to have as primary speed controller
// during the battery check led animation.
void initDynamicControlsDuringBatteryCheck()
{

        int sensorValue = analogRead(A0);                   // read the A0 pin value
        float voltage = sensorValue * (5.00 / 1600.00) * 2; // convert the value to a true voltage.
        Serial.print("voltage = ");
        Serial.print(voltage);
        Serial.println(" V");

        // Part 1 take initial readings
        readMpu(); // throw out first one.
        readMpu();
        float yaw = ypr[0] * 180 / M_PI;
        float pitch = ypr[1] * 180 / M_PI;
        float roll = ypr[2] * 180 / M_PI;

        blink(voltage); // blinks indicating approximate battery level

        // Part 2 take final readings post battery animation
        readMpu();
        float yaw_later = ypr[0] * 180 / M_PI;
        float pitch_later = ypr[1] * 180 / M_PI;
        float roll_later = ypr[2] * 180 / M_PI;

        // the one with the biggest difference is now the main speed controller
        float y_diff = abs(yaw - yaw_later);
        float r_diff = abs(roll - roll_later);

        if (DEBUG_OUTPUT)
        {
                Serial.print("yaw diff: ");
                Serial.print(y_diff);
                Serial.print("   roll diff: ");
                Serial.println(r_diff);
        }
        if (y_diff > r_diff)
                speedController = YAW;
        else
                speedController = ROLL;

        if (DEBUG_OUTPUT)
        {
                if (speedController == ROLL)
                        Serial.println("speedController: ROLL");
                else if (speedController == YAW)
                        Serial.println("speedController: YAW");
        }
}

// ================================================================
// ===                      INITIAL I2C SETUP                   ===
// ================================================================
void setupI2C()
{

        // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
#endif

        // initialize serial communication
        // (115200 chosen because it is required for Teapot Demo output, but it's
        // really up to you depending on your project)
        Serial.begin(115200);
        while (!Serial)
                ; // wait for Leonardo enumeration, others continue immediately

        // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
        // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
        // the baud timing being too misaligned with processor ticks. You must use
        // 38400 or slower in these cases, or use some kind of external separate
        // crystal solution for the UART timer.

        // initialize device
        Serial.println(F("Initializing I2C devices..."));
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);

        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        // wait for ready
        Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        //    while (Serial.available() && Serial.read()); // empty buffer
        // while (!Serial.available());                 // wait for data
        //   while (Serial.available() && Serial.read()); // empty buffer again

        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

        // make sure it worked (returns 0 if so)
        if (devStatus == 0)
        {
                // turn on the DMP, now that it's ready
                Serial.println(F("Enabling DMP..."));
                mpu.setDMPEnabled(true);

                // enable Arduino interrupt detection
                Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
                // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
                // mpuIntStatus = mpu.getIntStatus();
                mpuIntStatus = 1;
                // set our DMP Ready flag so the main loop() function knows it's okay to use it
                Serial.println(F("DMP ready! Waiting for first interrupt..."));
                dmpReady = true;

                // get expected DMP packet size for later comparison
                packetSize = mpu.dmpGetFIFOPacketSize();
        }
        else
        {
                // ERROR!
                // 1 = initial memory load failed
                // 2 = DMP configuration updates failed
                // (if it's going to break, usually the code will be 1)
                Serial.print(F("DMP Initialization failed (code "));
                Serial.print(devStatus);
                Serial.println(F(")"));
        }

        Serial.print(F("after the interupt wait..."));
        // configure LED for output
        pinMode(LED_PIN, OUTPUT);
}

void setupLedAnimations()
{
        randomSeed(analogRead(0)); // new random seed

        FastLED.show();

        Serial.begin(115200);
        Serial.println(" ");
        Serial.println("Ray's rave stave begin");

        // check if ram allocation of brushes and canvases was successful (painting will not work if unsuccessful, program should still run though)
        // this check is optional but helps to check if something does not work, especially on low ram chips like the Arduino Uno
        if (pixelcanvas.isvalid() == false)
                Serial.println("canvas allocation problem");
        else
                Serial.println("canvas allocation ok");

        if (pixelbrush.isvalid() == false)
                Serial.println("brush allocation problem");
        else
                Serial.println("brush allocation ok");

        brushcolor.h = random(255); // random color
        brushcolor.s = 255;         // full saturation
        brushcolor.v = 255;         // full brightness
}

unsigned long loopcounter; // count the loops, switch to next animation after a while
bool initialized = false;  // initialize the canvas & brushes in each loop when zero

void loop()
{
        // ledAnimationBundle();
        // blink();
        doUserInput(); // side steps the random show choice! FYI loops on itself

        if (randomShow > 7)
        {
                doUserInput();
        }
        else if (randomShow >= 4 && randomShow <= 6)
        {
                doSlowAnimations();
        }
        else
        {
                ledAnimationBundle();
        }
}

void doSlowAnimations()
{

        while (true)
        {
                initialized = false;
                twinkleStars();
                initialized = false;
                hueDemo();
                initialized = false;
                twoBrushColorMixing();
        }
}

void doUserInput()
{
        // initI2C();
        // readMpu();

        setupPropShield();
        readPropShield();

        if (YAW_ANIMATION_CONTROL)
        {
                float yaw = ypr[0] * 180 / M_PI;
                float pitch = ypr[1] * 180 / M_PI;
                float roll = ypr[2] * 180 / M_PI;
                programNotch = abs(yaw);
                Serial.print("programNotch: ");
                Serial.print(programNotch);
        }

        while (true)
        {
                doLeds();
        }
}

void doLeds()
{
        // initialized = false;

        // readMpu();
        readPropShield();

        float yaw = ypr[0] * 180 / M_PI;
        float pitch = ypr[1] * 180 / M_PI;
        float roll = ypr[2] * 180 / M_PI;

        if (YAW_ANIMATION_CONTROL)
        {
                int programNotchNow = abs(yaw);
                if (abs(programNotchNow - programNotch) > DIAL_SENSITIVITY)
                {

                        programNotch = abs(yaw);
                        animationNumber = (++animationNumber) % 3;
                        saveAnimationNum(animationNumber);
                        if (DEBUG_OUTPUT)
                        {
                                Serial.print("new programNotch: ");
                                Serial.print(programNotch);
                                Serial.print(" new animationNumber: ");
                                Serial.print(animationNumber);
                        }
                }
        }
        // raindbowPaint();
        //  bouncyBalls();
        //  initialized = false;
        // sparkler();
        // twoBrushColorMixing();
        if (DEBUG_OUTPUT)
        {
                // Serial.print("\nanimationNumber: ");
                // Serial.print(animationNumber);
                // Serial.print(" ");
        }

        dripReadMirror(); // DO NOT COMMIT: testing just dripRead
        // // Dynamic user input animations
        // if (animationNumber % ANIMATION_COUNT ==  0) {
        //         dripRead();
        // }
        // else if (animationNumber % ANIMATION_COUNT ==  1) {
        //         rainbowPaintRead();
        // }
        // else if (animationNumber % ANIMATION_COUNT ==  2) {
        //         sparklerFireRead(10, false);
        // }
        // else {
        //         sparklerRead();
        // }

        // Static fire animations
        // if (animationNumber % ANIMATION_COUNT ==  0) {
        //         sparklerFireRead(10, false);
        // }
        // else if (animationNumber % ANIMATION_COUNT ==  1) {
        //         sparklerFireRead(10, true);
        // }
        // else if (animationNumber % ANIMATION_COUNT ==  2) {
        //         sparklerFireRead(125, false);
        // }
        // else {
        //         sparklerFireRead(125, true);
        // }

        if (DEBUG_OUTPUT && YAW_ANIMATION_CONTROL)
        {
                Serial.print("next programNotch [ ");
                Serial.print(programNotch - DIAL_SENSITIVITY);
                Serial.print(" <-> ");
                Serial.print(programNotch + DIAL_SENSITIVITY);
                Serial.print(" ]\n");
        }
}

void saveAnimationNum(int number)
{
        EEPROM.write(ADDRESS_ANIMATION_NUM, number);
}

// Animation
void dripReadMirror()
{
        // Serial.print(F("in dripRead\n"));

        if (initialized == false) // initialize the brushes
        {
                pixelbrush.setFadeSpeed(90);
                pixelbrush.setFadein(false); // brightness will fade-in if set to true
                pixelbrush.setFadeout(true);
                pixelbrush.setBounce(false);
                pixelbrush.moveTo(0);

                pixelbrushM.setFadeSpeed(90);
                pixelbrushM.setFadein(false); // brightness will fade-in if set to true
                pixelbrushM.setFadeout(true);
                pixelbrushM.setBounce(false);
                pixelbrushM.moveTo(287);

                initialized = true;
                Serial.print("dripReadMirror init success");
        }

        // float yaw = ypr[0] * 180/M_PI;
        // float pitch = ypr[1] * 180/M_PI;
        // float roll = ypr[2] * 180/M_PI;

        float yaw = ypr[0];
        float pitch = ypr[1];
        float roll = ypr[2];

        updateSpeed(2);

        if (speed > 0 && lastSpeed < 0 ||
            speed < 0 && lastSpeed > 0)
        {
                brushcolor.h = random(255); // random color
        }

        pixelbrush.setSpeed(speed); // brush moving speed
        pixelbrushM.setSpeed(-speed);

        int curPos = pixelbrush.getPosition();
        if (curPos == 144)
        { // means pixelbrushBB must be at post 287
                pixelbrush.moveTo(0);
                // pixelbrush.setSpeed(-speed); // brush moving speed
                pixelbrushM.moveTo(287);
                // pixelbrushM.setSpeed(-speed); // brush moving speed
        }

        brushcolor.s = 255; // full saturation

        // int brightness = 125 - abs(pitch) * 2.8; //TODO configure the brightness according the secondaryController global var
        int brightness = int(abs(yaw)) % 120; // TODO configure the brightness according the secondaryController global var

        brushcolor.v = brightness; // random (peak) brighness

        pixelbrush.setColor(brushcolor);  // set new color to the brush
        pixelbrushM.setColor(brushcolor); // set new color to the brush

        FastLED.clear();

        pixelbrush.paint();  // paint the brush to the canvas (and update the brush, i.e. move it a little)
        pixelbrushM.paint(); // paint the brush to the canvas (and update the brush, i.e. move it a little)

        pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

        if (DEBUG_OUTPUT)
        {
                printInfo(brightness);
                // Serial.print("speed: ");
                // Serial.print(speed);
                // if(speedController == ROLL)
                //         Serial.println(" speedController: ROLL");
                // else if(speedController == YAW)
                //         Serial.println(" speedController: YAW");
                // Serial.print("\nbrightness: ");
                // Serial.print(brightness);
                // Serial.print("\n");
        }

        FastLED.show();
}

void dripRead()
{
        // Serial.print(F("in dripRead\n"));

        if (initialized == false) // initialize the brushes
        {
                initialized = true;
                pixelbrush.setFadeSpeed(90);
                pixelbrush.setFadein(false); // brightness will fade-in if set to true
                pixelbrush.setFadeout(true);
                pixelbrush.setBounce(false);
        }

        // float yaw = ypr[0] * 180/M_PI;
        // float pitch = ypr[1] * 180/M_PI;
        // float roll = ypr[2] * 180/M_PI;

        float yaw = ypr[0];
        float pitch = ypr[1];
        float roll = ypr[2];

        updateSpeed(2);

        if (speed > 0 && lastSpeed < 0 ||
            speed < 0 && lastSpeed > 0)
        {
                brushcolor.h = random(255); // random color
        }

        pixelbrush.setSpeed(speed); // brush moving speed

        brushcolor.s = 255; // full saturation

        // int brightness = 125 - abs(pitch) * 2.8; //TODO configure the brightness according the secondaryController global var
        int brightness = int(abs(yaw)) % 120; // TODO configure the brightness according the secondaryController global var

        brushcolor.v = brightness; // random (peak) brighness

        pixelbrush.setColor(brushcolor); // set new color to the bursh

        FastLED.clear();

        pixelbrush.paint();     // paint the brush to the canvas (and update the brush, i.e. move it a little)
        pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

        if (DEBUG_OUTPUT)
        {
                printInfo(brightness);
                // Serial.print("speed: ");
                // Serial.print(speed);
                // if(speedController == ROLL)
                //         Serial.println(" speedController: ROLL");
                // else if(speedController == YAW)
                //         Serial.println(" speedController: YAW");
                // Serial.print("\nbrightness: ");
                // Serial.print(brightness);
                // Serial.print("\n");
        }

        FastLED.show();
}

void printInfo(int brightness)
{
        // char buf[10];
        // sprintf(buf, "speed %d", speed)
        // Serial.println(buf);

        // Serial.print("speed: ");
        // Serial.print(speed);
        // Serial.print("   brightness: ");
        // Serial.print(brightness);
        // if(speedController == ROLL)
        //         Serial.print("   speedController: ROLL");
        // else if(speedController == YAW)
        //         Serial.println("   speedController: YAW");

        // float roll, pitch, yaw;
        // roll = filter.getRoll();
        // pitch = filter.getPitch();
        // yaw = filter.getYaw();
        // Serial.print(" yaw: ");
        // Serial.print(yaw);
        // Serial.print(" pitch: ");
        // Serial.print(pitch);
        // Serial.print(" roll: ");
        // Serial.println(roll);

        // Serial.print("\n");
}

void printInfo2(int brightness)
{

        Serial.print("speed: ");
        Serial.print(speed);
        Serial.print("   brightness: ");
        Serial.print(brightness);
        if (speedController == ROLL)
                Serial.print("   speedController: ROLL");
        else if (speedController == YAW)
                Serial.println("   speedController: YAW");

        float roll, pitch, yaw;
        roll = filter.getRoll();
        pitch = filter.getPitch();
        yaw = filter.getYaw();
        Serial.print(" yaw: ");
        Serial.print(yaw);
        Serial.print(" pitch: ");
        Serial.print(pitch);
        Serial.print(" roll: ");
        Serial.println(roll);

        Serial.print("\n");
}

void ledAnimationBundle()
{
        while (true)
        {
                initialized = false;
                rainbowPaint();
                initialized = false;
                sparkler(); // fast shooting white balls
                initialized = false;
                twinkleStars(); // not good for motion control
                initialized = false;
                chaser(); // low surface area dark chaser
                initialized = false;
                hueDemo(); // maybe brightness?
                initialized = false;
                speedTrails(); // great for motion roll
                initialized = false;
                bouncyBalls();
                initialized = false;
                twoBrushColorMixing();
        }
}

void rainbowBlink()
{
        Serial.println(F("rainbow blink"));
        // the brush moves along the strip, leaving a colorful rainbow trail
        for (loopcounter = 0; loopcounter < 100; loopcounter++)
        {
                static unsigned int hue = 0; // color hue to set to brush
                // CHSV brushcolor; //HSV color definition

                if (initialized == false) // initialize the brushes
                {
                        initialized = true;
                        pixelbrush.setSpeed(random(200) + 200); // brush moving speed
                        pixelbrush.setFadeSpeed(90);
                        pixelbrush.setFadein(false); // brightness will fade-in if set to true
                        pixelbrush.setFadeout(true);
                        pixelbrush.setBounce(false);
                }

                hue++;
                brushcolor.h = hue / 3; // divide by 3 to slow down color fading
                brushcolor.s = 255;     // full saturation
                brushcolor.v = 125;     // full brightness

                pixelbrush.setColor(brushcolor); // set new color to the bursh

                FastLED.clear();

                pixelbrush.paint();     // paint the brush to the canvas (and update the brush, i.e. move it a little)
                pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

                FastLED.show();
        }
}

//---------------------
// RAINBOW PAINT (aka nyan cat)
//---------------------
// Animation
void rainbowPaintRead()
{
        // Serial.println(F("in rainbow paint read"));
        // the brush moves along the strip, leaving a colorful rainbow trail
        // for(loopcounter = 0; loopcounter<duration; loopcounter++)
        // {
        static unsigned int hue = 0; // color hue to set to brush

        if (initialized == false) // initialize the brushes
        {
                initialized = true;
                pixelbrush.setSpeed(speed); // brush moving speed
                pixelbrush.setFadeSpeed(90);
                pixelbrush.setFadein(false); // brightness will fade-in if set to true
                pixelbrush.setFadeout(true);
                pixelbrush.setBounce(false);
        }

        // float yaw = ypr[0] * 180/M_PI;
        // float pitch = ypr[1] * 180/M_PI;
        // float roll = ypr[2] * 180/M_PI;

        float yaw = ypr[0];
        float pitch = ypr[1];
        float roll = ypr[2];

        updateSpeed(2);
        pixelbrush.setSpeed(speed); // brush moving speed

        hue++;
        brushcolor.h = hue / 3; // divide by 3 to slow down color fading
        brushcolor.s = 255;     // full saturation

        // int brightness = 125 - abs(pitch) * 2.8; //TODO configure the brightness according the secondaryController global var
        int brightness = int(abs(yaw)) % 120; // TODO configure the brightness according the secondaryController global var

        brushcolor.v = brightness; // random (peak) brighness
        // brushcolor.v = 255; //full brightness

        pixelbrush.setColor(brushcolor); // set new color to the bursh

        FastLED.clear();

        pixelbrush.paint();     // paint the brush to the canvas (and update the brush, i.e. move it a little)
        pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

        if (DEBUG_OUTPUT)
        {
                printInfo(brightness);
                // Serial.print("speed: ");
                // Serial.print(speed);
                // if(speedController == ROLL)
                //         Serial.println(" speedController: ROLL");
                // else if(speedController == YAW)
                //         Serial.println(" speedController: YAW");
                // Serial.print("\nbrightness: ");
                // Serial.print(brightness);
                // Serial.print("\n");
        }

        FastLED.show();
        // }
}
//---------------------
// RAINBOW PAINT (aka nyan cat)
//---------------------
// Animation
void rainbowPaint()
{
        Serial.println(F("rainbow paint"));
        // the brush moves along the strip, leaving a colorful rainbow trail
        for (loopcounter = 0; loopcounter < duration; loopcounter++)
        {
                static unsigned int hue = 0; // color hue to set to brush
                // CHSV brushcolor; //HSV color definition

                if (initialized == false) // initialize the brushes
                {
                        initialized = true;
                        pixelbrush.setSpeed(random(200) + 200); // brush moving speed
                        pixelbrush.setFadeSpeed(90);
                        pixelbrush.setFadein(false); // brightness will fade-in if set to true
                        pixelbrush.setFadeout(true);
                        pixelbrush.setBounce(false);
                }

                hue++;
                brushcolor.h = hue / 3; // divide by 3 to slow down color fading
                brushcolor.s = 255;     // full saturation
                brushcolor.v = 255;     // full brightness

                pixelbrush.setColor(brushcolor); // set new color to the bursh

                FastLED.clear();

                pixelbrush.paint();     // paint the brush to the canvas (and update the brush, i.e. move it a little)
                pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

                FastLED.show();
        }
}

void rainbowPaintWithInput()
{
        Serial.println(F("rainbow paint with input"));
        // the brush moves along the strip, leaving a colorful rainbow trail
        for (loopcounter = 0; loopcounter < duration; loopcounter++)
        {
                readPropShield(); // still want the averages to update

                static unsigned int hue = 0; // color hue to set to brush
                // CHSV brushcolor; //HSV color definition

                if (initialized == false) // initialize the brushes
                {
                        initialized = true;
                        pixelbrush.setSpeed(random(100) + 300); // brush moving speed
                        pixelbrush.setFadeSpeed(90);
                        pixelbrush.setFadein(false); // brightness will fade-in if set to true
                        pixelbrush.setFadeout(true);
                        pixelbrush.setBounce(false);
                }

                hue++;
                brushcolor.h = hue / 3; // divide by 3 to slow down color fading
                brushcolor.s = 255;     // full saturation
                // brushcolor.v = 255; //full brightness
                brushcolor.v = 155; // full brightness

                // float yaw = ypr[0];
                // int brightness = (int(abs(yaw)) % 120) + 50;
                // brushcolor.v = brightness;

                pixelbrush.setColor(brushcolor); // set new color to the bursh

                FastLED.clear();

                pixelbrush.paint();     // paint the brush to the canvas (and update the brush, i.e. move it a little)
                pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

                FastLED.show();
        }

        spinning_animation = false;
}

void updateSpeed(int multiplier)
{
        // float yaw = ypr[0] * 180/M_PI;
        // float pitch = ypr[1] * 180/M_PI;
        // float roll = ypr[2] * 180/M_PI;

        float yaw = ypr[0];
        float pitch = ypr[1];
        float roll = ypr[2];

        if (speedController == ROLL)
        {
                lastSpeed = speed;
                speed = roll * multiplier;
        }
        else
        {
                lastSpeed = speed;
                speed = yaw * multiplier;
        }
}

// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 50, suggested range 20-100
#define COOLING 55

// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKING 120
bool gReverseDirection = false;

void sparklerFireReadOg()
{
        // Array of temperature readings at each simulation cell
        static byte heat[NUM_LEDS];

        // Step 1.  Cool down every cell a little
        for (int i = 0; i < NUM_LEDS; i++)
        {
                heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
        }

        // Step 2.  Heat from each cell drifts 'up' and diffuses a little
        for (int k = NUM_LEDS - 1; k >= 2; k--)
        {
                heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
        }

        // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
        if (random8() < SPARKING)
        {
                int y = random8(7);
                heat[y] = qadd8(heat[y], random8(160, 255));
        }

        // Step 4.  Map from heat cells to LED colors
        for (int j = 0; j < NUM_LEDS; j++)
        {
                CRGB color = HeatColor(heat[j]);
                int pixelnumber;
                if (gReverseDirection)
                {
                        pixelnumber = (NUM_LEDS - 1) - j;
                }
                else
                {
                        pixelnumber = j;
                }
                leds[pixelnumber] = color;
        }

        FastLED.show(); // display this frame
}

void sparklerFireRead(uint8_t fireBrightness, bool reverse)
{
        // Array of temperature readings at each simulation cell
        static byte heat[NUM_LEDS];

        // Step 1.  Cool down every cell a little
        for (int i = 0; i < NUM_LEDS; i++)
        {
                heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
        }

        // Step 2.  Heat from each cell drifts 'up' and diffuses a little
        for (int k = NUM_LEDS - 1; k >= 2; k--)
        {
                heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
        }

        // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
        if (random8() < SPARKING)
        {
                int y = random8(7);
                heat[y] = qadd8(heat[y], random8(100, 255));
        }

        // Step 4.  Map from heat cells to LED colors
        for (int j = 0; j < NUM_LEDS; j++)
        {
                // Scale the heat value from 0-255 down to 0-240
                // for best results with color palettes.
                byte colorindex = scale8(heat[j], 240);
                CRGB color = ColorFromPalette(CRGBPalette16(CRGB::Black, CRGB::Red, CRGB::Orange, CRGB::BlueViolet), colorindex, fireBrightness);
                if (reverse)
                {
                        color = ColorFromPalette(CRGBPalette16(CRGB::BlueViolet, CRGB::Orange, CRGB::Red, CRGB::Black), colorindex, fireBrightness);
                }

                int pixelnumber;
                if (gReverseDirection)
                {
                        pixelnumber = (NUM_LEDS - 1) - j;
                }
                else
                {
                        pixelnumber = j;
                }
                leds[pixelnumber] = color;
        }

        FastLED.show(); // display this frame
}

// SPARKLER: a brush seeding sparkles
//  Animation
void sparklerRead()
{
        // Serial.print(F("in sparklerRead\n"));

        // float yaw = ypr[0] * 180/M_PI;
        // float pitch = ypr[1] * 180/M_PI;
        // float roll = ypr[2] * 180/M_PI;

        float yaw = ypr[0];
        float pitch = ypr[1];
        float roll = ypr[2];

        // updateSpeed(roll * 5 + 600);
        if (initialized == false)
        {
                initialized = true;
                updateSpeed(2);
                pixelbrush.setSpeed(speed);
                // if(speed > 0)
                //   pixelbrush.setSpeed(600);
                // else
                //   pixelbrush.setSpeed(-600);
                pixelbrush.setFadeout(true); // sparkles fade in
                pixelbrush.setFadein(true);  // and fade out immediately after reaching the set brightness
        }

        int saturation = 255 - abs(yaw) * 2.8;

        // set a new brush color in each loop
        brushcolor.h = random(255); // random color
        brushcolor.s = saturation;  // random(130); //random but low saturation, giving white-ish sparkles
        int brightness = random(125);
        brushcolor.v = brightness; // random (peak) brighness

        pixelbrush.setColor(brushcolor);
        pixelbrush.setFadeSpeed(random(100) + 150); // set a new fadespeed with some randomness

        FastLED.clear();

        pixelbrush.paint();     // paint the brush to the canvas (and update the brush, i.e. move it a little)
        pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

        if (DEBUG_OUTPUT)
        {
                printInfo(brightness);
                // Serial.print("speed: ");
                // Serial.print(speed);
                // if(speedController == ROLL)
                //         Serial.println(" speedController: ROLL");
                // else if(speedController == YAW)
                //         Serial.println(" speedController: YAW");
                // Serial.print("\nsaturation: ");
                // Serial.print(saturation);
                // Serial.print("\n");
        }

        FastLED.show();
}

// SPARKLER: a brush seeding sparkles
//  Animation
void sparkler()
{
        Serial.println(F("sparkler"));
        for (loopcounter = 0; loopcounter < duration; loopcounter++)
        {

                // CHSV brushcolor;

                if (initialized == false)
                {
                        initialized = true;
                        pixelbrush.setSpeed(600);
                        pixelbrush.setFadeout(true); // sparkles fade in
                        pixelbrush.setFadein(true);  // and fade out immediately after reaching the set brightness
                }

                // set a new brush color in each loop
                brushcolor.h = random(255); // random color
                brushcolor.s = random(130); // random but low saturation, giving white-ish sparkles
                brushcolor.v = random(200); // random (peak) brighness

                pixelbrush.setColor(brushcolor);
                pixelbrush.setFadeSpeed(random(100) + 150); // set a new fadespeed with some randomness

                FastLED.clear();

                pixelbrush.paint();     // paint the brush to the canvas (and update the brush, i.e. move it a little)
                pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

                FastLED.show();
        }
}

//---------------------
// TWINKLE STARS
//---------------------
// Animation
void twinkleStars()
{
        Serial.println(F("twinkler stars"));
        // brush set to random positions and painting a fading star
        for (loopcounter = 0; loopcounter < duration; loopcounter++)
        {

                // CHSV brushcolor;

                if (initialized == false)
                {
                        initialized = true;
                        pixelbrush.setSpeed(0);      // do not move automatically
                        pixelbrush.setFadein(true);  // fade in
                        pixelbrush.setFadeout(true); // and fade out
                }

                if (rand() % 100 == 0) // at a random interval, move the brush to paint a new pixel (brush only paints a new pixel once)
                {
                        brushcolor.h = rand();
                        brushcolor.s = random(40);       // set low saturation, almost white
                        brushcolor.v = random(200) + 20; // set random brightness
                        pixelbrush.setColor(brushcolor);
                        pixelbrush.moveTo(random(NUM_LEDS));     // move the brush to a new, random pixel
                        pixelbrush.setFadeSpeed(random(10) + 5); // set random fade speed, minimum of 5
                }

                // add a background color by setting all pixels to a color (instead of clearing all pixels):
                fill_solid(leds, NUM_LEDS, CRGB(1, 0, 6)); // color in RGB: dark blue

                pixelbrush.paint();     // paint the brush to the canvas
                pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

                FastLED.show();
        }
}

//-------------
// CHASER
//-------------
// Animation
void chaser()
{
        Serial.println(F("chaser"));

        // two brushes chasing each other, one painting the pixel in a color, the other one painting 'black' (acting on the same canvas)

        while (true) // create a loop with an additional brush (is deleted automatically once the loop finishes)
        {

                // create an additional brush, painting on the same canvas as the globally defined brush
                FastLEDPainterBrush pixelbrush2 = FastLEDPainterBrush(&pixelcanvas); // crete brush, linked to the canvas to paint to

                if (pixelbrush2.isvalid() == false)
                        Serial.println(F("brush2 allocation problem"));
                else
                        Serial.println(F("brush2 allocation ok"));

                for (loopcounter = 0; loopcounter < duration; loopcounter++)
                {

                        if (initialized == false)
                        {
                                // CHSV brushcolor;

                                initialized = true;

                                brushcolor.h = random(255); // choose random color once
                                brushcolor.s = 255;         // full staturation
                                brushcolor.v = 150;

                                // initialize the first brush to move and paint a color, no fading
                                pixelbrush.setSpeed(300); // moving speed was 900 but too fast for teensy
                                pixelbrush.setColor(brushcolor);
                                pixelbrush.setFadeout(false); // deactivate fade-out (was activated in last animation)
                                pixelbrush.setFadein(false);  // deactivate fade-in
                                pixelbrush2.moveTo(0);        // move the brush to pixel 0
                                // initialize the second brush to move at the same speed but starting at a different position (default position is 0)
                                brushcolor.v = 0;          // zero intensity = black
                                pixelbrush2.setSpeed(300); // moving speed was 900 but too fast for teensy
                                pixelbrush2.setColor(brushcolor);
                                pixelbrush2.moveTo(2 * NUM_LEDS / 3); // move the brush
                        }

                        FastLED.clear();

                        pixelbrush.paint();     // apply the paint of the first brush to the canvas (and update the brush)
                        pixelbrush2.paint();    // apply the paint of the second brush to the canvas (and update the brush)
                        pixelcanvas.transfer(); // transfer the canvas to the FastLED

                        FastLED.show();
                }
                break; // quit the while loop immediately (and delete the created brush)
        }
}

//------------------------------
// HUE FADER: demo of hue fading
//------------------------------
// Animation
void hueDemo()
{
        Serial.println(F("hue demo"));
        // hue fading can be done in two ways: change the color moving the shortest distance around the colorwheel (setFadeHueNear)
        // or intentionally moving around the colorwheel choosing the long way (setFadeHueFar)
        // two brushes move along the strip in different speeds, each painting a different color that the canvas will then fade to
        // a new color is set when the first brush passes pixel 0
        // both brushes act on the same canvas

        while (true) // create a loop with an additional brush (is deleted automatically once the loop finishes)
        {

                // create an additional brush, painting on the same canvas as the globally defined brush
                FastLEDPainterBrush pixelbrush2 = FastLEDPainterBrush(&pixelcanvas);

                if (pixelbrush2.isvalid() == false)
                        Serial.println(F("brush2 allocation problem"));
                else
                        Serial.println(F("brush2 allocation ok"));

                pixelcanvas.clear(); // clear the canvas

                for (loopcounter = 0; loopcounter < duration; loopcounter++)
                {

                        static unsigned int lastposition = 0; // to detect zero crossing only once (brush may stay at pixel zero for some time since it uses sub-pixel resolution movement)

                        if (pixelbrush.getPosition() == 0 && lastposition > 0)
                                initialized = false; // choose new color & speed if brush reaches pixel 0

                        lastposition = pixelbrush.getPosition(); // save current position for next position check

                        if (initialized == false)
                        {
                                initialized = true;

                                // CHSV brushcolor;

                                brushcolor.h = random(255); // random color
                                brushcolor.s = 255;         // full saturation
                                brushcolor.v = 130;         // medium brightness

                                pixelbrush.setSpeed(random(150) + 150);   // random movement speed
                                pixelbrush.setFadeSpeed(random(10) + 20); // set random fading speed
                                pixelbrush.setColor(brushcolor);          // update the color of the brush
                                pixelbrush.setFadeHueNear(true);          // fade using the near path on the colorcircle

                                // second brush paints on the same canvas
                                brushcolor.h = random(255);
                                pixelbrush2.setSpeed(random(150) + 150);
                                pixelbrush2.setFadeSpeed(random(10) + 20);
                                pixelbrush2.setColor(brushcolor);
                                pixelbrush2.setFadeHueNear(true); // fade using the near path on the colorcircle
                                // pixelbrush.setFadeHueFar(true); //fade using the far path on the colorcircle (if both are set, this path is chosen)
                                pixelbrush2.setBounce(true); // bounce this brush at the end of the strip
                        }

                        FastLED.clear();

                        pixelbrush.paint();     // apply the paint of the first brush to the canvas (and update the brush)
                        pixelbrush2.paint();    // apply the paint of the second brush to the canvas (and update the brush)
                        pixelcanvas.transfer(); // transfer the canvas to the FastLED

                        FastLED.show();
                }
                break; // quit the while loop immediately (and delete the created brush)
        }
}

//------------------------------
// SPEEDTRAILS
//------------------------------
// three brushes painting on one canvas, all following each other at the same speed, painting fading pixels
// Animation
void speedTrails()
{
        Serial.println(F("speed trails"));
        while (true) // create a loop with two additional brushes (are deleted automatically once the loop finishes)
        {
                int brushspeed = 300; // moving speed was 900 but too fast for teensy

                // create additional brushes, painting on the same canvas as the globally defined brush
                FastLEDPainterBrush pixelbrush2 = FastLEDPainterBrush(&pixelcanvas); // crete brush, linked to the canvas to paint to
                FastLEDPainterBrush pixelbrush3 = FastLEDPainterBrush(&pixelcanvas); // crete brush, linked to the canvas to paint to

                if (pixelbrush2.isvalid() == false)
                        Serial.println(F("brush2 allocation problem"));
                else
                        Serial.println(F("brush2 allocation ok"));

                if (pixelbrush3.isvalid() == false)
                        Serial.println(F("brush3 allocation problem"));
                else
                        Serial.println(F("brush3 allocation ok"));

                for (loopcounter = 0; loopcounter < duration; loopcounter++)
                {

                        // CHSV brushcolor;

                        if (initialized == false) // initialize the brushes
                        {
                                initialized = true;
                                brushcolor.h = 0;
                                brushcolor.s = 0;   // make it white
                                brushcolor.v = 150; // medium brightness
                                pixelbrush.setColor(brushcolor);
                                pixelbrush.setSpeed(brushspeed);
                                pixelbrush.setFadeSpeed(250);     // fast fading (255 is max.)
                                pixelbrush.setFadeHueNear(false); // deactivate hue fading, was activated in last animation
                                pixelbrush.setFadeout(true);
                                pixelbrush.moveTo(0); // move brush to zero

                                // second brush
                                brushcolor.h = 0; // red
                                brushcolor.s = 250;
                                brushcolor.v = 150;
                                pixelbrush2.setSpeed(brushspeed);
                                pixelbrush2.setFadeSpeed(220);
                                pixelbrush2.setFadeout(true);
                                pixelbrush2.setColor(brushcolor);
                                pixelbrush2.moveTo(NUM_LEDS / 3); // move it up one third of the strip

                                // third brush
                                brushcolor.h = 28; // yellow
                                brushcolor.s = 255;
                                brushcolor.v = 100;
                                pixelbrush3.setSpeed(brushspeed);
                                pixelbrush3.setFadeSpeed(190);
                                pixelbrush3.setFadeout(true);
                                pixelbrush3.setColor(brushcolor);
                                pixelbrush3.moveTo(2 * NUM_LEDS / 3); // move it up two thirds of the strip
                        }

                        FastLED.clear();

                        pixelbrush.paint();     // apply the paint of the first brush to the canvas (and update the brush)
                        pixelbrush2.paint();    // apply the paint of the second brush to the canvas (and update the brush)
                        pixelbrush3.paint();    // apply the paint of the third brush to the canvas (and update the brush)
                        pixelcanvas.transfer(); // transfer the canvas to the FastLED (and update i.e. fade pixels)

                        FastLED.show();
                }
                break; // quit the while loop immediately (and delete the created brush)
        }
}

//-------------
// BOUNCY BALLS
//-------------
// three brushes painting on one canvas, attracted to the zero pixel as if by gravity
// Animation
void bouncyBalls()
{
        Serial.println(F("bouncy balls"));
        while (true) // create a loop with two additional brushes (are deleted automatically once the loop finishes)
        {

                // create additional brushes, painting on the same canvas as the globally defined brush
                FastLEDPainterBrush pixelbrush2 = FastLEDPainterBrush(&pixelcanvas); // crete brush, linked to the canvas to paint to
                FastLEDPainterBrush pixelbrush3 = FastLEDPainterBrush(&pixelcanvas); // crete brush, linked to the canvas to paint to

                if (pixelbrush2.isvalid() == false)
                        Serial.println(F("brush2 allocation problem"));
                else
                        Serial.println(F("brush2 allocation ok"));

                if (pixelbrush3.isvalid() == false)
                        Serial.println(F("brush3 allocation problem"));
                else
                        Serial.println(F("brush3 allocation ok"));

                byte skipper = 0;

                for (loopcounter = 0; loopcounter < duration; loopcounter++)
                {

                        if (initialized == false) // initialize the brushes
                        {
                                initialized = true;

                                // CHSV brushcolor;

                                brushcolor.h = 20;  // orange
                                brushcolor.s = 240; // almost full saturation
                                brushcolor.v = 150; // medium brightness

                                // first brush
                                pixelbrush.setSpeed(0); // zero initial speed
                                pixelbrush.setFadeSpeed(150);
                                pixelbrush.setFadeout(true);
                                pixelbrush.setColor(brushcolor);
                                pixelbrush.moveTo(NUM_LEDS - 1); // move to end of the strip
                                pixelbrush.setBounce(true);      // bounce if either end of the strip is reached

                                // second brush
                                brushcolor.h = 220;      // pink
                                pixelbrush2.setSpeed(0); // zero initial speed
                                pixelbrush2.setFadeSpeed(190);
                                pixelbrush2.setFadeout(true);
                                pixelbrush2.setColor(brushcolor);
                                pixelbrush2.moveTo(NUM_LEDS / 3); // move to one third of the strip
                                pixelbrush2.setBounce(true);

                                brushcolor.h = 70; // green-ish (pure green is 85 or 255/3)
                                pixelbrush3.setSpeed(0);
                                pixelbrush3.setFadeSpeed(220);
                                pixelbrush3.setFadeout(true);
                                pixelbrush3.setColor(brushcolor);
                                pixelbrush3.moveTo(2 * NUM_LEDS / 3);
                                pixelbrush3.setBounce(true);
                        }

                        // apply some gravity force that accelerates the painters (i.e. add speed in negative direction = towards zero pixel)
                        //   if (skipper % 5 == 0) //only apply gravity at some interval to make it slower on fast processors
                        //   {
                        // read current speed of each brush and speed it up in negative direction (towards pixel zero)
                        pixelbrush.setSpeed(pixelbrush.getSpeed() - 5);
                        pixelbrush2.setSpeed(pixelbrush2.getSpeed() - 5);
                        pixelbrush3.setSpeed(pixelbrush3.getSpeed() - 5);
                        //  }
                        //  skipper++;

                        FastLED.clear();

                        pixelbrush.paint();     // apply the paint of the first brush to the canvas (and update the brush)
                        pixelbrush2.paint();    // apply the paint of the second brush to the canvas (and update the brush)
                        pixelbrush3.paint();    // apply the paint of the third brush to the canvas (and update the brush)
                        pixelcanvas.transfer(); // transfer the canvas to the FastLED (and update i.e. fade pixels)

                        FastLED.show();
                }
                break; // quit the while loop immediately (and delete the created brush)
        }
}

void bouncyBallsWithInput()
{
        Serial.println(F("bouncy balls"));
        // while(true) //create a loop with two additional brushes (are deleted automatically once the loop finishes)
        // for(loopcounter = 0; loopcounter<duration; loopcounter++)
        // {

        // readPropShield(); // still want the averages to update

        // //create additional brushes, painting on the same canvas as the globally defined brush
        // FastLEDPainterBrush pixelbrush2 = FastLEDPainterBrush(&pixelcanvas); //crete brush, linked to the canvas to paint to
        // FastLEDPainterBrush pixelbrush3 = FastLEDPainterBrush(&pixelcanvas); //crete brush, linked to the canvas to paint to

        // if (pixelbrush2.isvalid() == false) Serial.println(F("brush2 allocation problem"));
        // else Serial.println(F("brush2 allocation ok"));

        // if (pixelbrush3.isvalid() == false) Serial.println(F("brush3 allocation problem"));
        // else Serial.println(F("brush3 allocation ok"));

        byte skipper = 0;

        for (loopcounter = 0; loopcounter < duration; loopcounter++)
        {

                readPropShield(); // still want the averages to update

                if (initialized == false) // initialize the brushes
                {
                        initialized = true;

                        // CHSV brushcolor;

                        brushcolor.h = 20;  // orange
                        brushcolor.s = 240; // almost full saturation
                        // brushcolor.v = 150; //medium brightness
                        brushcolor.v = 110;

                        // first brush
                        pixelbrush.setSpeed(0); // zero initial speed
                        pixelbrush.setFadeSpeed(150);
                        pixelbrush.setFadeout(true);
                        pixelbrush.setColor(brushcolor);
                        pixelbrush.moveTo(NUM_LEDS - 1); // move to end of the strip
                        pixelbrush.setBounce(true);      // bounce if either end of the strip is reached

                        // second brush
                        brushcolor.h = 220;        // pink
                        pixelbrushBB2.setSpeed(0); // zero initial speed
                        pixelbrushBB2.setFadeSpeed(190);
                        pixelbrushBB2.setFadeout(true);
                        pixelbrushBB2.setColor(brushcolor);
                        pixelbrushBB2.moveTo(NUM_LEDS / 3); // move to one third of the strip
                        pixelbrushBB2.setBounce(true);

                        brushcolor.h = 70; // green-ish (pure green is 85 or 255/3)
                        pixelbrushBB3.setSpeed(0);
                        pixelbrushBB3.setFadeSpeed(220);
                        pixelbrushBB3.setFadeout(true);
                        pixelbrushBB3.setColor(brushcolor);
                        pixelbrushBB3.moveTo(2 * NUM_LEDS / 3);
                        pixelbrushBB3.setBounce(true);
                }

                // apply some gravity force that accelerates the painters (i.e. add speed in negative direction = towards zero pixel)
                if (skipper % 5 == 0) // only apply gravity at some interval to make it slower on fast processors
                {
                        // read current speed of each brush and speed it up in negative direction (towards pixel zero)
                        pixelbrush.setSpeed(pixelbrush.getSpeed() - 5);
                        pixelbrushBB2.setSpeed(pixelbrushBB2.getSpeed() - 5);
                        pixelbrushBB3.setSpeed(pixelbrushBB3.getSpeed() - 5);
                }
                skipper++;

                FastLED.clear();

                pixelbrush.paint();     // apply the paint of the first brush to the canvas (and update the brush)
                pixelbrushBB2.paint();  // apply the paint of the second brush to the canvas (and update the brush)
                pixelbrushBB3.paint();  // apply the paint of the third brush to the canvas (and update the brush)
                pixelcanvas.transfer(); // transfer the canvas to the FastLED (and update i.e. fade pixels)

                FastLED.show();
        }
        // break; //quit the while loop immediately (and delete the created brush)
        // }
        FastLED.clear();
        spinning_animation = false;
}

//---------------------
// TWO-BRUSH-COLORMIXING
//---------------------
// two brushes moving around randomly paint on their individual canvas, resulting in colors being mixed
// Animation
void twoBrushColorMixing()
{
        Serial.println(F("two brush mix"));
        while (true) // create a loop with two additional brushes (are deleted automatically once the loop finishes)
        {

                // create an additional canvas for the second brush (needs to be created before the brush)
                FastLEDPainterCanvas pixelcanvas2 = FastLEDPainterCanvas(NUM_LEDS);

                // create additional brush, painting on the second canvas
                FastLEDPainterBrush pixelbrush2 = FastLEDPainterBrush(&pixelcanvas2);

                if (pixelcanvas2.isvalid() == false)
                        Serial.println("canvas2 allocation problem");
                else
                        Serial.println("canvas2 allocation ok");

                if (pixelbrush2.isvalid() == false)
                        Serial.println(F("brush2 allocation problem"));
                else
                        Serial.println(F("brush2 allocation ok"));

                for (loopcounter = 0; loopcounter < duration; loopcounter++)
                {

                        // CHSV brushcolor;
                        static bool firstrun = true;

                        brushcolor.s = 255; // full color saturation
                        brushcolor.v = 100; // medium-low brightness

                        if (initialized == false) // initialize the brushes
                        {
                                initialized = true;

                                brushcolor.h = 8;

                                // setup the first brush
                                pixelbrush.setSpeed(-350); // was 750
                                pixelbrush.setSpeedlimit(400);
                                pixelbrush.setFadeSpeed(random(80) + 50);
                                pixelbrush.setFadeout(true);
                                pixelbrush.setFadein(true);
                                pixelbrush.setColor(brushcolor);
                                pixelbrush.moveTo(random(NUM_LEDS));
                                pixelbrush.setBounce(true);

                                // setup the second brush
                                brushcolor.h = 160;
                                pixelbrush2.setSpeed(200); // was 600
                                pixelbrush2.setSpeedlimit(400);
                                pixelbrush2.setFadeSpeed(random(80) + 50);
                                pixelbrush2.setFadeout(true);
                                pixelbrush2.setFadein(true);
                                pixelbrush2.setColor(brushcolor);
                                pixelbrush2.moveTo(random(NUM_LEDS));
                                pixelbrush2.setBounce(true);
                        }

                        if (rand() % 10) // slowly (and randomly) change hue of brushes
                        {
                                brushcolor = pixelbrush.getColor();
                                brushcolor.h += random(3) - 1; // randomly change hue a little ( ± random(1))
                                pixelbrush.setColor(brushcolor);

                                brushcolor = pixelbrush2.getColor();
                                brushcolor.h += random(3) - 1; // randomly change hue a little ( ± random(1))
                                pixelbrush2.setColor(brushcolor);
                        }

                        // slowly change speed of both brushes
                        pixelbrush.setSpeed(pixelbrush.getSpeed() + random(6) - 3);   // means speed = currentspeed ± random(3)
                        pixelbrush2.setSpeed(pixelbrush2.getSpeed() + random(6) - 3); // means speed = currentspeed ± random(3)

                        FastLED.clear(); // remove any previously applied paint

                        pixelbrush.paint();  // apply the paint of the first brush to its assigned canvas (and update the brush)
                        pixelbrush2.paint(); // apply the paint of the second brush to  its assigned canvas (and update the brush)

                        pixelcanvas.transfer();  // transfer the first canvas to the FastLED
                        pixelcanvas2.transfer(); // transfer the sedonc canvas to the FastLED (adding colors, rather than overwriting colors)

                        FastLED.show();
                }
                break; // quit the while loop immediately (and delete the created brush)
        }
}

void initI2C()
{
        // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
#endif

        // initialize serial communication
        // (115200 chosen because it is required for Teapot Demo output, but it's
        // really up to you depending on your project)
        Serial.begin(115200);
        rainbowBlink();
        // while (!Serial); // wait for Leonardo enumeration, others continue immediately

        // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
        // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
        // the baud timing being too misaligned with processor ticks. You must use
        // 38400 or slower in these cases, or use some kind of external separate
        // crystal solution for the UART timer.

        // initialize device
        Serial.println(F("Initializing I2C devices..."));
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);

        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        // wait for ready
        Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        // while (Serial.available() && Serial.read()); // empty buffer
        // while (!Serial.available());                 // wait for data
        // while (Serial.available() && Serial.read()); // empty buffer again

        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

        // make sure it worked (returns 0 if so)
        if (devStatus == 0)
        {
                // turn on the DMP, now that it's ready
                Serial.println(F("Enabling DMP..."));
                mpu.setDMPEnabled(true);

                // enable Arduino interrupt detection
                Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
                attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
                mpuIntStatus = mpu.getIntStatus();

                // set our DMP Ready flag so the main loop() function knows it's okay to use it
                Serial.println(F("DMP ready! Waiting for first interrupt..."));
                dmpReady = true;

                // get expected DMP packet size for later comparison
                packetSize = mpu.dmpGetFIFOPacketSize();
        }
        else
        {
                // ERROR!
                // 1 = initial memory load failed
                // 2 = DMP configuration updates failed
                // (if it's going to break, usually the code will be 1)
                Serial.print(F("DMP Initialization failed (code "));
                Serial.print(devStatus);
                Serial.println(F(")"));
        }

        // configure LED for output
        pinMode(LED_PIN, OUTPUT);
}

void initI2C_custom()
{
        Serial.print(F("here"));
        // if programming failed, don't try to do anything
        if (!dmpReady)
                return;
        Serial.print(F("heredddddd"));
        // wait for MPU interrupt or extra packet(s) available
        // while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
        //   }

        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        Serial.print(F("here2"));
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024)
        {
                // reset so we can continue cleanly
                mpu.resetFIFO();
                Serial.println(F("FIFO overflow!"));

                // otherwise, check for DMP data ready interrupt (this should happen frequently)
        }

        Serial.print(F("here3"));
}

void setupPropShield()
{
        Serial.begin(9600);
        imu.begin();
        filter.begin(100);
}

void readPropShield()
{
        float ax, ay, az;
        float gx, gy, gz;
        float mx, my, mz;
        float roll, pitch, yaw;

        if (imu.available())
        {
                // Read the motion sensors
                imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

                // Update the SensorFusion filter
                filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

                // print the heading, pitch and roll
                roll = filter.getRoll();
                pitch = filter.getPitch();
                yaw = filter.getYaw();
                ypr[0] = yaw;
                ypr[1] = roll; // pitch and roll seemed to be swapped between ic's
                ypr[2] = pitch;

                // handle the roll rate calc
                //     rollHistory.push(pitch);
                // if (millis() - time >= 250 && rollHistory.isFull()) {
                if (millis() - time >= 250)
                {
                        time = millis();
                        // float avg = 0.0;
                        // float sum = 0;
                        // // the following ensures using the right type for the index variable
                        // using index_t = decltype(rollHistory)::index_t;
                        // for (index_t i = 0; i < rollHistory.size(); i++) {
                        //         // avg += buffer[i] / (float)buffer.size();
                        //         // roll_rate += rollHistory[i] / (float)rollHistory.size();
                        //         sum += rollHistory[i];
                        // }

                        printInfo2(brushcolor.v);

                        // roll_rate = sum / rollHistory.size();
                        // Serial.print("Average is ");
                        roll_rate = pitch;
                        Serial.print("RATE is ");
                        Serial.print(abs(old_roll_rate - roll_rate));
                        Serial.print(" spinning: ");
                        Serial.println(spinning_animation);

                        if (!spinning_animation && abs(old_roll_rate - roll_rate) > ROLL_THRESHOLD)
                        {
                                // should switch to standard animations
                                spinning_animation = true;
                                // rainbowPaintWithInput();
                                bouncyBallsWithInput();
                        }
                        old_roll_rate = roll_rate;
                }

                //     Serial.print("Orientation [ypr]: ");
                //     Serial.print(yaw);
                //     Serial.print(" ");
                //     Serial.print(pitch);
                //     Serial.print(" ");
                //     Serial.println(roll);
        }
}

void readMpu()
{

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        // wait for correct available data length, should be a VERY short wait
        // Serial.print(F("in readMpu if first"));
        while (fifoCount < packetSize && mpuIntStatus < 1)
                fifoCount = mpu.getFIFOCount();
        // Serial.print(F("after readMpu() while "));

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // HERE IS WHERE THE READS ACTUALLY HAPPEN
#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display YAW PITCH AND ROLL angles in degrees
        mpu.dmpGetQuaternion(&qc, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &qc);
        mpu.dmpGetYawPitchRoll(ypr, &qc, &gravity);
        float yaw = ypr[0] * 180 / M_PI;
        float pitch = ypr[1] * 180 / M_PI;
        float roll = ypr[2] * 180 / M_PI;

        if (DEBUG_OUTPUT)
        {
                Serial.print("\typr\t");
                Serial.print(yaw);
                Serial.print("\t");
                Serial.print(pitch);
                Serial.print("\t");
                Serial.println(roll);
        }
#endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
}

float readRollOld()
{
        Serial.print(F("in readRoll()"));
        Serial.print(mpuIntStatus);
        mpuIntStatus = 1;
        if (0x02)
                Serial.print(F("0x02"));
        if (mpuIntStatus & 0x02 || true)
        {
                // wait for correct available data length, should be a VERY short wait
                Serial.print(F("in readRoll if first"));
                while (fifoCount < packetSize)
                        fifoCount = mpu.getFIFOCount();
                Serial.print(F("after readRoll() while"));

                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);

                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;

                // HERE IS WHERE THE READS ACTUALLY HAPPEN
#ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display YAW PITCH AND ROLL angles in degrees
                mpu.dmpGetQuaternion(&qc, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &qc);
                mpu.dmpGetYawPitchRoll(ypr, &qc, &gravity);
                Serial.print("ypr\t");
                float yaw = ypr[0] * 180 / M_PI;
                Serial.print(yaw);
                Serial.print("\t");
                float pitch = ypr[1] * 180 / M_PI;
                Serial.print(pitch);
                Serial.print("\t");
                float roll = ypr[2] * 180 / M_PI;
                Serial.println(roll);
#endif

                // blink LED to indicate activity
                blinkState = !blinkState;
                digitalWrite(LED_PIN, blinkState);

                return roll;
        }
}

void drip()
{
        boolean backward = true;
        // static signed int speed = 0;
        // for(loopcounter = 0; loopcounter < duration; loopcounter++)
        while (true)
        {
                static unsigned int hue = 0; // color hue to set to brush

                // CHSV brushcolor; //HSV color definition

                if (initialized == false) // initialize the brushes
                {
                        initialized = true;
                        pixelbrush.setFadeSpeed(90);
                        pixelbrush.setFadein(false); // brightness will fade-in if set to true
                        pixelbrush.setFadeout(true);
                        pixelbrush.setBounce(false);
                }

                if (backward)
                {
                        speed -= 1;
                        brushcolor.h = 200;
                }
                else
                {
                        speed += 1;
                        brushcolor.h = 100;
                }

                if (speed > 300)
                {
                        backward = true;
                }
                else if (speed < -300)
                        backward = false;

                Serial.print("speed");
                Serial.print(speed);
                Serial.print("\n");

                pixelbrush.setSpeed(speed); // brush moving speed

                // hue++;
                // brushcolor.h = hue; //divide by 3 to slow down color fading
                brushcolor.s = 255; // full saturation
                brushcolor.v = 255; // full brightness

                pixelbrush.setColor(brushcolor); // set new color to the bursh

                FastLED.clear();

                pixelbrush.paint();     // paint the brush to the canvas (and update the brush, i.e. move it a little)
                pixelcanvas.transfer(); // transfer (add) the canvas to the FastLED

                FastLED.show();
        }
}
