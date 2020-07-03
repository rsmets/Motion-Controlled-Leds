void blink(float voltage) {
        int blinkCount = 0;
        if(voltage > 4.2)
                blinkCount = 5;
        else if(voltage > 4.0)
                blinkCount = 4;
        else if(voltage > 3.8)
                blinkCount = 3;
        else if(voltage > 3.6)
                blinkCount = 2;
        else if(voltage > 3.4)
                blinkCount = 1;

        for(int i = 0; i < blinkCount; i++) {
                pixelbrush.setFadeSpeed(90);
                pixelbrush.setFadein(false); //brightness will fade-in if set to true
                pixelbrush.setFadeout(true);
                pixelbrush.setBounce(false);

                pixelbrush.setSpeed(0); //brush moving speed

                brushcolor.s = 255; //full saturation
                int brightness = 255;
                brushcolor.v = brightness; //random (peak) brighness

                pixelbrush.setColor(brushcolor); //set new color to the bursh

                FastLED.clear();
                FastLED.show();
                delay(200);

                pixelbrush.paint(); //paint the brush to the canvas (and update the brush, i.e. move it a little)
                pixelcanvas.transfer(); //transfer (add) the canvas to the FastLED

                FastLED.show();
                delay(200);
        }
}

void batteryCheck() {
        int sensorValue = analogRead(A0); //read the A0 pin value
        float voltage = sensorValue * (5.00 / 1600.00) * 2; //convert the value to a true voltage.
        Serial.print("voltage = ");
        Serial.print(voltage);
        Serial.println(" V");

        blink(voltage); //blinks indicating approximate battery level
}

// Allows one to orient the stave in position one would like to have as primary speed controller
// during the battery check led animation.
void initDynamicControlsDuringBatteryCheck() {

        int sensorValue = analogRead(A0); //read the A0 pin value
        float voltage = sensorValue * (5.00 / 1600.00) * 2; //convert the value to a true voltage.
        Serial.print("voltage = ");
        Serial.print(voltage);
        Serial.println(" V");

        // Part 1 take initial readings
        readMpu(); //throw out first one.
        readMpu();
        float yaw = ypr[0] * 180/M_PI;
        float pitch = ypr[1] * 180/M_PI;
        float roll = ypr[2] * 180/M_PI;

        blink(voltage); //blinks indicating approximate battery level

        // Part 2 take final readings post battery animation
        readMpu();
        float yaw_later = ypr[0] * 180/M_PI;
        float pitch_later = ypr[1] * 180/M_PI;
        float roll_later = ypr[2] * 180/M_PI;

        // the one with the biggest difference is now the main speed controller
        float y_diff = abs(yaw - yaw_later);
        float r_diff = abs(roll - roll_later);

        if(DEBUG_OUTPUT) {
                Serial.print("yaw diff: ");
                Serial.print(y_diff);
                Serial.print("   roll diff: ");
                Serial.println(r_diff);
        }
        if(y_diff > r_diff)
                speedController = YAW;
        else
                speedController = ROLL;

        if(DEBUG_OUTPUT) {
                if(speedController == ROLL)
                        Serial.println("speedController: ROLL");
                else if(speedController == YAW)
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
        while (!Serial); // wait for Leonardo enumeration, others continue immediately

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
        if (devStatus == 0) {
                // turn on the DMP, now that it's ready
                Serial.println(F("Enabling DMP..."));
                mpu.setDMPEnabled(true);

                // enable Arduino interrupt detection
                Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
                //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
                //mpuIntStatus = mpu.getIntStatus();
                mpuIntStatus = 1;
                // set our DMP Ready flag so the main loop() function knows it's okay to use it
                Serial.println(F("DMP ready! Waiting for first interrupt..."));
                dmpReady = true;

                // get expected DMP packet size for later comparison
                packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
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
        randomSeed(analogRead(0)); //new random seed

        FastLED.show();

        Serial.begin(115200);
        Serial.println(" ");
        Serial.println("Ray's rave stave begin");

        //check if ram allocation of brushes and canvases was successful (painting will not work if unsuccessful, program should still run though)
        //this check is optional but helps to check if something does not work, especially on low ram chips like the Arduino Uno
        if (pixelcanvas.isvalid() == false) Serial.println("canvas allocation problem");
        else Serial.println("canvas allocation ok");


        if (pixelbrush.isvalid() == false) Serial.println("brush allocation problem");
        else Serial.println("brush allocation ok");

        brushcolor.h = random(255); //random color
        brushcolor.s = 255; //full saturation
        brushcolor.v = 255; //full brightness
}

unsigned long loopcounter; //count the loops, switch to next animation after a while
bool initialized = false; //initialize the canvas & brushes in each loop when zero

void loop()
{
//ledAnimationBundle();
        //blink();
        doUserInput();
        if( randomShow >7)
        {
                doUserInput();
        }
        else if(randomShow>=4 && randomShow<=6)
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

        while(true)
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

        if(YAW_ANIMATION_CONTROL) {
                float yaw = ypr[0] * 180/M_PI;
                float pitch = ypr[1] * 180/M_PI;
                float roll = ypr[2] * 180/M_PI;
                programNotch = abs(yaw);
                Serial.print("programNotch: ");
                Serial.print(programNotch);
        }

        while(true)
        {
                doLeds();
        }

}

void doLeds()
{
        initialized = false;

        // readMpu();
        readPropShield();

        float yaw = ypr[0] * 180/M_PI;
        float pitch = ypr[1] * 180/M_PI;
        float roll = ypr[2] * 180/M_PI;

        if(YAW_ANIMATION_CONTROL)
        {
                int programNotchNow = abs(yaw);
                if(abs(programNotchNow - programNotch) > DIAL_SENSITIVITY) {

                        programNotch = abs(yaw);
                        animationNumber = (++animationNumber)%3;
                        saveAnimationNum(animationNumber);
                        if(DEBUG_OUTPUT) {
                                Serial.print("new programNotch: ");
                                Serial.print(programNotch);
                                Serial.print(" new animationNumber: ");
                                Serial.print(animationNumber);
                        }
                }
        }
        //raindbowPaint();
        // bouncyBalls();
        // initialized = false;
        //sparkler();
        //twoBrushColorMixing();
        if(DEBUG_OUTPUT) {
                Serial.print("\nanimationNumber: ");
                Serial.print(animationNumber);
                Serial.print(" ");
        }

        if (animationNumber % ANIMATION_COUNT ==  0) {
                dripRead();
        }
        else if (animationNumber % ANIMATION_COUNT ==  1) {
                rainbowPaintRead();
        }
        else {
                sparklerRead();
        }

        if(DEBUG_OUTPUT && YAW_ANIMATION_CONTROL) {
                Serial.print("next programNotch [ ");
                Serial.print(programNotch - DIAL_SENSITIVITY);
                Serial.print(" <-> ");
                Serial.print(programNotch + DIAL_SENSITIVITY);
                Serial.print(" ]\n");
        }
}

void saveAnimationNum(int number) {
        EEPROM.write(ADDRESS_ANIMATION_NUM, number);
}

void dripRead()
{
        Serial.print(F("in dripRead\n"));

        if (initialized == false) //initialize the brushes
        {
                initialized = true;
                pixelbrush.setFadeSpeed(90);
                pixelbrush.setFadein(false); //brightness will fade-in if set to true
                pixelbrush.setFadeout(true);
                pixelbrush.setBounce(false);
        }

        // float yaw = ypr[0] * 180/M_PI;
        // float pitch = ypr[1] * 180/M_PI;
        // float roll = ypr[2] * 180/M_PI;

        float yaw = ypr[0];
        float pitch = ypr[1];
        float roll = ypr[2];  

        updateSpeed(5);

        if(speed > 0 && lastSpeed < 0 ||
           speed < 0 && lastSpeed > 0) {
                brushcolor.h = random(255); //random color
        }

        pixelbrush.setSpeed(speed); //brush moving speed

        brushcolor.s = 255; //full saturation

        int brightness = 255 - abs(pitch) * 2.8; //TODO configure the brightness according the secondaryController global var

        brushcolor.v = brightness; //random (peak) brighness

        pixelbrush.setColor(brushcolor); //set new color to the bursh

        FastLED.clear();

        pixelbrush.paint(); //paint the brush to the canvas (and update the brush, i.e. move it a little)
        pixelcanvas.transfer(); //transfer (add) the canvas to the FastLED

        if (DEBUG_OUTPUT) {
                Serial.print("speed: ");
                Serial.print(speed);
                if(speedController == ROLL)
                        Serial.println(" speedController: ROLL");
                else if(speedController == YAW)
                        Serial.println(" speedController: YAW");
                Serial.print("\nbrightness: ");
                Serial.print(brightness);
                Serial.print("\n");
        }

        FastLED.show();
}

void ledAnimationBundle()
{
        while(true)
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
                hueDemo(); //maybe brightness?
                initialized = false;
                speedTrails(); //great for motion roll
                initialized = false;
                bouncyBalls();
                initialized = false;
                twoBrushColorMixing();
        }

}

void rainbowBlink()
{
        Serial.println(F("rainbow blink"));
        //the brush moves along the strip, leaving a colorful rainbow trail
        for(loopcounter = 0; loopcounter<100; loopcounter++)
        {
                static unsigned int hue = 0; //color hue to set to brush
                //CHSV brushcolor; //HSV color definition


                if (initialized == false) //initialize the brushes
                {
                        initialized = true;
                        pixelbrush.setSpeed(random(200) + 200); //brush moving speed
                        pixelbrush.setFadeSpeed(90);
                        pixelbrush.setFadein(false); //brightness will fade-in if set to true
                        pixelbrush.setFadeout(true);
                        pixelbrush.setBounce(false);
                }

                hue++;
                brushcolor.h = hue / 3; //divide by 3 to slow down color fading
                brushcolor.s = 255; //full saturation
                brushcolor.v = 255; //full brightness

                pixelbrush.setColor(brushcolor); //set new color to the bursh

                FastLED.clear();

                pixelbrush.paint(); //paint the brush to the canvas (and update the brush, i.e. move it a little)
                pixelcanvas.transfer(); //transfer (add) the canvas to the FastLED

                FastLED.show();
        }

}

//---------------------
//RAINBOW PAINT (aka nyan cat)
//---------------------
void rainbowPaintRead()
{
        Serial.println(F("in rainbow paint read"));
        //the brush moves along the strip, leaving a colorful rainbow trail
        // for(loopcounter = 0; loopcounter<duration; loopcounter++)
        // {
        static unsigned int hue = 0; //color hue to set to brush


        if (initialized == false) //initialize the brushes
        {
                initialized = true;
                pixelbrush.setSpeed(speed); //brush moving speed
                pixelbrush.setFadeSpeed(90);
                pixelbrush.setFadein(false); //brightness will fade-in if set to true
                pixelbrush.setFadeout(true);
                pixelbrush.setBounce(false);
        }

        // float yaw = ypr[0] * 180/M_PI;
        // float pitch = ypr[1] * 180/M_PI;
        // float roll = ypr[2] * 180/M_PI;

        float yaw = ypr[0];
        float pitch = ypr[1];
        float roll = ypr[2];  

        updateSpeed(5);
        pixelbrush.setSpeed(speed); //brush moving speed

        hue++;
        brushcolor.h = hue / 3; //divide by 3 to slow down color fading
        brushcolor.s = 255; //full saturation
        int brightness = 255 - abs(pitch) * 2.8; //TODO configure the brightness according the secondaryController global var
        brushcolor.v = brightness; //random (peak) brighness
        //brushcolor.v = 255; //full brightness

        pixelbrush.setColor(brushcolor); //set new color to the bursh

        FastLED.clear();

        pixelbrush.paint(); //paint the brush to the canvas (and update the brush, i.e. move it a little)
        pixelcanvas.transfer(); //transfer (add) the canvas to the FastLED

        if (DEBUG_OUTPUT) {
                Serial.print("speed: ");
                Serial.print(speed);
                if(speedController == ROLL)
                        Serial.println(" speedController: ROLL");
                else if(speedController == YAW)
                        Serial.println(" speedController: YAW");
                Serial.print("\nbrightness: ");
                Serial.print(brightness);
                Serial.print("\n");
        }

        FastLED.show();
        // }
}
//---------------------
//RAINBOW PAINT (aka nyan cat)
//---------------------
void rainbowPaint()
{
        Serial.println(F("rainbow paint"));
        //the brush moves along the strip, leaving a colorful rainbow trail
        for(loopcounter = 0; loopcounter<duration; loopcounter++)
        {
                static unsigned int hue = 0; //color hue to set to brush
                //CHSV brushcolor; //HSV color definition


                if (initialized == false) //initialize the brushes
                {
                        initialized = true;
                        pixelbrush.setSpeed(random(200) + 200); //brush moving speed
                        pixelbrush.setFadeSpeed(90);
                        pixelbrush.setFadein(false); //brightness will fade-in if set to true
                        pixelbrush.setFadeout(true);
                        pixelbrush.setBounce(false);
                }

                hue++;
                brushcolor.h = hue / 3; //divide by 3 to slow down color fading
                brushcolor.s = 255; //full saturation
                brushcolor.v = 255; //full brightness

                pixelbrush.setColor(brushcolor); //set new color to the bursh

                FastLED.clear();

                pixelbrush.paint(); //paint the brush to the canvas (and update the brush, i.e. move it a little)
                pixelcanvas.transfer(); //transfer (add) the canvas to the FastLED

                FastLED.show();
        }
}


//SPARKLER: a brush seeding sparkles
void sparklerRead() {}

//SPARKLER: a brush seeding sparkles
void sparkler() {}

//---------------------
//TWINKLE STARS
//---------------------
void twinkleStars() {}

//-------------
//CHASER
//-------------
void chaser(){}

//------------------------------
//HUE FADER: demo of hue fading
//------------------------------
void hueDemo(){}

//------------------------------
//SPEEDTRAILS
//------------------------------
//three brushes painting on one canvas, all following each other at the same speed, painting fading pixels
void speedTrails(){}

//-------------
//BOUNCY BALLS
//-------------
//three brushes painting on one canvas, attracted to the zero pixel as if by gravity
void bouncyBalls(){}

//---------------------
//TWO-BRUSH-COLORMIXING
//---------------------
//two brushes moving around randomly paint on their individual canvas, resulting in colors being mixed
void twoBrushColorMixing(){}

float readRollOld(){}

void drip(){}
