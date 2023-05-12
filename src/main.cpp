#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <MPU9250.h>

#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels
#define OLED_RESET 5     // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D
const int USER_INPUT_BUTTON = 13;
const int BUTTON_STATUS = 0;
int imu_status;
float axb, ayb, azb, axs, ays, azs, ax, ay, az, gx, gy, gz,
    gxb, gyb, gzb, hxb, hyb, hzb, set_gxb, set_gyb, set_gzb,
    hxs, hys, hzs, mx, my, mz = 0;

// Declare SPI object and CS pin here
MPU9250 IMU(Wire, 0x68);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Test
//---------------------------------------------------- [ DISPLAY FUNCTIONS ] ----------------------------------------------------//

void display_values_table()
{
  Serial.println("--------------- ACCELEROMOTOR DATA --------           --------------- GYROSCOPE DATA ---------------           --------------- MAGOMETER DATA ---------------");
  Serial.print("aX : ");
  Serial.print(ax);
  Serial.print(' ');
  Serial.print("\taY : ");
  Serial.print(ay);
  Serial.print(' ');
  Serial.print("\taZ : ");
  Serial.print(az);

  Serial.print("\t\tgX : ");
  Serial.print(gx);
  Serial.print(' ');
  Serial.print("\tgY : ");
  Serial.print(gy);
  Serial.print(' ');
  Serial.print("\tgZ : ");
  Serial.print(gz);

  Serial.print("\t\tmX : ");
  Serial.print(mx);
  Serial.print(' ');
  Serial.print("\tmY : ");
  Serial.print(my);
  Serial.print(' ');
  Serial.print("\tmZ : ");
  Serial.println(mz);
}
void display_values_serial()
{
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print(',');
  Serial.print(mx);
  Serial.print(',');
  Serial.print(my);
  Serial.print(',');
  Serial.println(mz);
}
void setup()
{
  //---------------------------------------------------- [ ESTABLISH SERIAL COMMUNICATION AND CHECK IMU ] ----------------------------------------------------//
  Serial.begin(115200);
  Serial.println("PROGRAM STARTED");
  pinMode(USER_INPUT_BUTTON, INPUT);

  // initialize OLED display with I2C address 0x3C
  if (!oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  oled.setTextColor(WHITE);
  oled.setTextSize(1);

  oled.clearDisplay();
  oled.setCursor(35, 5);
  oled.println("CHECKING");
  oled.setCursor(30, 30);
  oled.println("IMU STATUS");
  oled.setCursor(60, 50);
  oled.display();

  Serial.println("-------------[ CHECKING IMU STATUS ]-------------\n");
  oled.clearDisplay();
  // Not sure why this is needed..... But we do

  // If something is wrong with the IMU, IMU.begin() will return a -1 and an error message will pop up on the serial monitor
  imu_status = IMU.begin();
  if (imu_status < 0)
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(imu_status);

    oled.setCursor(0, 10);
    oled.println("IMU FAILED");
    oled.setCursor(0, 30);
    oled.println("CHECK IMU WIRING");
    oled.setCursor(0, 50);
    oled.println("OR TRY POWER CYCLING");
    oled.display();

    while (1)
    {
    }
  }
  else
  {
    Serial.println("-------------[ IMU IS OPERATIONAL ]-------------\n");
    oled.setCursor(50, 10);
    oled.println("IMU ");
    oled.setCursor(30, 30);
    oled.println("OPERATIONAL");
    oled.display();
    delay(1200);
  }

  //---------------------------------------------------- [ CALIBRATE ACCELEROMOTOR ] ----------------------------------------------------//
  Serial.println("========================================================================================================================");
  Serial.println("\n-------------[ CALIBRATING ACCELEROMOTOR ]-------------\n");

  oled.clearDisplay();
  oled.setCursor(30, 10);
  oled.println("CALIBRATING");
  oled.setCursor(30, 40);
  oled.println("ACCELEROMOTOR");
  oled.display();

  delay(1200);

  Serial.println("Orientate IMU to desiered INITAL position to calibrate all SIX axes\n");
  delay(1200);
  Serial.println("To initiate ACCELEROMOTOR calibration press [ ENTER ]...");

  oled.clearDisplay();
  oled.setCursor(30, 0);
  oled.println("PRESS ENTER");

  oled.setCursor(30, 20);
  oled.println("TO CONTINUE");
  oled.display();

  // Calibration won't start until user inputs 'ENTER' in the serial monitor
  while (true)
  {
    // Checks serial monitor for incoming data
    if (Serial.available() > 0 || digitalRead(USER_INPUT_BUTTON) == HIGH)
    {
      // If user presses 'ENTER' break out of while loop
      if (Serial.read() == '\n' || Serial.read() == '\r' || digitalRead(USER_INPUT_BUTTON) == HIGH)
      {
        oled.clearDisplay();

        oled.setCursor(25, 0);
        oled.println("ACCELEROMOTOR ");

        oled.setCursor(30, 20);
        oled.println("CALIBRATION");

        oled.setCursor(30, 35);
        oled.println("STARTING IN");

        oled.setCursor(60, 50);

        Serial.println("\nCALIBRATION STARTING IN...");
        oled.display();
        break;
      }
    }

    oled.print(".");
    oled.display();

    Serial.print(".");
    delay(100);
  }
  delay(1200);

  // This is where the actual calibration takes place. Orientate the IMU sensor in SIX different directions AND MAKE SURE IT'S STABLE
  for (int n = 1; n <= 6; n++)
  {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.print("CURRENT CALIBRATION");
    oled.setCursor(0, 20);
    oled.print("ITERATION(S) :");
    oled.setCursor(100, 20);
    oled.println(n);
    // oled.display();

    Serial.print("\nCurrent calibration iteration(s) : ");
    Serial.print(n);
    Serial.println(" \\ 6\n");

    // Countdown timer
    for (int i = 5; i > 0; i--)
    {
      oled.clearDisplay();
      oled.setCursor(0, 40);
      oled.println("STARTING IN : ");
      oled.setCursor(100, 40);
      oled.println(i);
      oled.display();

      Serial.print(i);
      Serial.println("...");
      delay(1000);
    }

    // delay(2000);

    Serial.println("\n-------------[ CALIBRATION IN PROGRESS ]-------------");

    oled.clearDisplay();
    oled.setCursor(35, 0);
    oled.println("CALIBRATION ");
    oled.setCursor(50, 20);
    oled.println("IN");
    oled.setCursor(35, 50);
    oled.println("PROGRESS");
    oled.display();

    IMU.calibrateAccel();

    oled.clearDisplay();
    oled.setCursor(35, 0);
    oled.println("CHANGE ");

    oled.setCursor(45, 20);
    oled.println("IMU");

    oled.setCursor(35, 50);
    oled.println("POSITION");
    oled.display();
    delay(1000);

    Serial.println("\n-------------[ Change IMU posistion ]-------------");
  }
  // oled.clearDisplay();
  // oled.setCursor(35, 0);
  // oled.println("CURRENT ");
  // oled.setCursor(30, 20);
  // oled.println("CALIBRATION");
  // oled.setCursor(30, 35);
  // oled.println("ITERATION");
  // oled.setCursor(60, 50);
  // oled.println("5");
  // oled.display();
  delay(1000);
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print("CALIBRATION COMPLETE");
  oled.display();
  Serial.println("\n-------------[ Calibrating accelerometer COMPLETE ]-------------\n");

  // Get Scale factor values
  // Scale factors are the ratio between the measured output and the change in the 'sense' input
  // More info about scale factors can be found here : https://www.edn.com/evaluating-inertial-measurement-units/
  axs = IMU.getAccelScaleFactorX();
  ays = IMU.getAccelScaleFactorY();
  azs = IMU.getAccelScaleFactorZ();

  // Biased Factors
  axb = IMU.getAccelBiasX_mss();
  ayb = IMU.getAccelBiasY_mss();
  azb = IMU.getAccelBiasZ_mss();

  // Display scale/bias values
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("X SCALE FACTOR : ");
  oled.setCursor(100, 0);
  oled.println(axs);
  oled.setCursor(0, 25);
  oled.println("Y SCALE FACTOR : ");
  oled.setCursor(100, 25);
  oled.println(ays);
  oled.setCursor(0, 50);
  oled.println("Z SCALE FACTOR : ");
  oled.setCursor(100, 50);
  oled.println(azs);
  oled.display();

  Serial.println("\n-------------[ SCALE FACTORS ]-------------\n");
  Serial.print("X_Scale Factor : ");
  Serial.print(axs);
  Serial.print(" Y_Scale Factor : ");
  Serial.print(ays);
  Serial.print(" Z_Scale Factor : ");
  Serial.println(azs);

  delay(2000);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("X BIAS FACTOR : ");
  oled.setCursor(100, 0);
  oled.println(axb);
  oled.setCursor(0, 25);
  oled.println("Y BIAS FACTOR : ");
  oled.setCursor(100, 25);
  oled.println(ayb);
  oled.setCursor(0, 50);
  oled.println("Z BIAS FACTOR : ");

  oled.setCursor(100, 50);
  oled.println(azb);
  oled.display();

  Serial.println("\n-------------[ BIAS VALUES]-------------\n");

  Serial.print("X_Biased : ");
  Serial.print(axb);
  Serial.print(" Y_Biased : ");
  Serial.print(ayb);
  Serial.print(" Z_Biased : ");
  Serial.println(azb);

  // Store scale/bias values into an array
  const float accelBias[3] = {axb, ayb, azb};
  const float accelFactor[3] = {axs, ays, azs};

  // Set the scale/bias values accordingly
  Serial.println("\n-------------[ IMU VALUES SET TO]-------------\n");
  IMU.setAccelCalX(accelBias[0], accelFactor[0]);
  IMU.setAccelCalY(accelBias[1], accelFactor[1]);
  IMU.setAccelCalZ(accelBias[2], accelFactor[2]);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("IMU SCALE/BIAS VALUES");
  oled.setCursor(30, 25);
  oled.println("ARE STORED");
  oled.display();

  delay(2000);

  // Display
  Serial.print("Ax_Bias : ");
  Serial.print(accelBias[0]);
  Serial.print(" | Ax_Accel : ");
  Serial.println(accelFactor[0]);
  Serial.print("Ay_Bias : ");
  Serial.print(accelBias[1]);
  Serial.print(" | Ay_Accel : ");
  Serial.println(accelFactor[1]);
  Serial.print("Az_Bias : ");
  Serial.print(accelBias[2]);
  Serial.print(" | Az_Accel : ");
  Serial.println(accelFactor[2]);

  // delay(3000);

  Serial.println("\n========================================================================================================================");
  delay(2000);

  //---------------------------------------------------- [ CALIBRATE GYRO ] ----------------------------------------------------//
  oled.clearDisplay();
  oled.setCursor(30, 0);
  oled.println("CALIBRATING");
  oled.setCursor(30, 20);
  oled.println("GYROSCOPE");
  oled.setCursor(30, 35);
  oled.println("PRESS ENTER");
  oled.setCursor(30, 50);
  oled.println("TO CONTINUE");
  oled.display();

  Serial.println("\n-------------[ CALIBRATING GYROSCOPE ]-------------");
  Serial.println("\nPress [ ENTER ] to continue...");

  // Calibration won't start until user inputs 'ENTER' in the serial monitor
  while (true)
  {
    // Checks serial monitor for incoming data
    if (Serial.available() > 0 || digitalRead(USER_INPUT_BUTTON) == HIGH)
    {
      // If user presses 'ENTER' break out of while loop
      if (Serial.read() == '\n' || Serial.read() == '\r' || digitalRead(USER_INPUT_BUTTON) == HIGH)
      {
        oled.clearDisplay();
        oled.setCursor(30, 0);
        oled.println("PLACE IMU");
        oled.setCursor(15, 20);
        oled.println("ON SOLID SURFACE");
        oled.display();

        Serial.println("\n\nPlace IMU sensor on solid surface and DON'T TOUCH IT");
        delay(2000);

        oled.clearDisplay();
        oled.setCursor(30, 20);
        oled.println("CALIBRATION");
        oled.setCursor(30, 35);
        oled.println("STARTING IN");
        oled.setCursor(60, 50);

        Serial.println("\nCALIBRATION STARTING IN...\n");

        break;
      }
    }
    oled.print(".");
    oled.display();
    Serial.print(".");
    delay(100);
  }
  oled.clearDisplay();

  // Countdown timer
  for (int i = 5; i > 0; i--)
  {
    oled.setCursor(50, 20);
    oled.println(i);
    oled.display();

    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }

  Serial.println("CALIBRATION STARTED");
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("CALIBRATION STARTED");

  // Where the actual Gyro calibration takes place
  if (IMU.calibrateGyro() == true)
  {
    oled.clearDisplay();
    oled.setCursor(30, 0);
    oled.println("GYROSCOPE");
    oled.setCursor(30, 20);
    oled.println("CALIBRATION");
    oled.setCursor(30, 45);
    oled.println("COMPLETE");
    oled.display();
    Serial.println("\nGYROSCOPE CALIBRATION COMPLETE\n");

    delay(2000);

    // "Grab" bias values
    gxb = IMU.getGyroBiasX_rads();
    gyb = IMU.getGyroBiasY_rads();
    gzb = IMU.getGyroBiasZ_rads();

    // Set bias values
    IMU.setGyroBiasX_rads(gxb);
    IMU.setGyroBiasY_rads(gyb);
    IMU.setGyroBiasZ_rads(gzb);

    // Print out bias values

    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("GYRO BIAS X : ");
    oled.setCursor(100, 0);
    oled.println(gxb);
    oled.setCursor(0, 25);
    oled.println("GYRO BIAS Y : ");
    oled.setCursor(100, 25);
    oled.println(gyb);
    oled.setCursor(0, 50);
    oled.println("GYRO BIAS Z : ");
    oled.setCursor(100, 50);
    oled.println(gzb);

    Serial.println("---- GYRO BIAS VALUES SET ----\n");
    Serial.print("GXB : ");
    Serial.print(gxb);
    Serial.print(',');
    Serial.print(" GYB : ");
    Serial.print(gyb);
    Serial.print(',');
    Serial.print(" GZB : ");
    Serial.println(gzb);

    delay(2000);
  }

  //---------------------------------------------------- [ CALIBRATE MAGNETOMETER ] ----------------------------------------------------//
  oled.clearDisplay();
  oled.setCursor(30, 0);
  oled.println("CALIBRATING");
  oled.setCursor(30, 20);
  oled.println("MAGNETOMETER");
  oled.setCursor(30, 35);
  oled.println("PRESS ENTER");
  oled.setCursor(30, 50);
  oled.println("TO CONTINUE");
  oled.display();

  Serial.println("\nPress [ ENTER ] to calibrate megnetometer...");
  // Calibration won't start until user inputs 'ENTER' in the serial monitor
  while (true)
  {
    // Checks serial monitor for incoming data
    if (Serial.available() > 0 || digitalRead(USER_INPUT_BUTTON) == HIGH)
    {
      // If user presses 'ENTER' break out of while loop
      if (Serial.read() == '\n' || Serial.read() == '\r' || digitalRead(USER_INPUT_BUTTON) == HIGH)
      {
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.println("PROCESS WILL TAKE ~60-80");
        oled.setCursor(0, 20);
        oled.println("SECONDS TO COMPLETE");
        oled.setCursor(0, 40);
        oled.println("PREPARE TO MAKE");
        oled.setCursor(0, 55);
        oled.println("FIGURE 8 MOTION");
        oled.display();

        Serial.println("\n-------------[ CALIBRATING MAGNETOMETER WILL TAKE ~60-80 SECONDS TO COMPLETE...]-------------");
        // delay(1500);
        Serial.println("\n-------------[ SLOWLY AND CONTINUOUSLY MAKE A FIGURE 8 MOTION DURING CALIBRATION ]-------------");
        delay(2000);

        Serial.println("\nCALIBRATION STARTING IN...\n");
        // oled.clearDisplay();
        // oled.setCursor(30, 20);
        // oled.println("CALIBRATION");
        // oled.setCursor(30, 35);
        // oled.println("STARTING IN");
        // oled.setCursor(60, 50);

        break;
      }
    }
    oled.print(".");
    oled.display();
    Serial.print(".");
    delay(100);
  }

  // Countdown timer
  for (int i = 5; i > 0; i--)
  {
    oled.clearDisplay();
    oled.setCursor(0, 40);
    oled.println("STARTING IN : ");
    oled.setCursor(100, 40);
    oled.println(i);
    oled.display();

    Serial.print(i);
    Serial.println("...");
    delay(1000);

    // oled.setCursor(50, 20);
    // oled.println(i);
    // oled.display();

    // Serial.print(i);
    // Serial.println("...");
    // delay(1000);
  }

  oled.clearDisplay();
  oled.setCursor(30, 0);
  oled.println("CALIBRATION IN PROGRESS");
  oled.display();
  Serial.println("\n-------------[ CALIBRATION IN PROGRESS ]-------------");

  if (IMU.calibrateMag() == true)
  {
    // Gather magnetometer bias/scale factors
    hxb = IMU.getMagBiasX_uT();
    hyb = IMU.getMagBiasY_uT();
    hzb = IMU.getMagBiasZ_uT();
    hxs = IMU.getMagScaleFactorX();
    hys = IMU.getMagScaleFactorY();
    hzs = IMU.getMagScaleFactorZ();

    // Display bias/scale factors
    Serial.println("\nMAGNETOMETER CALIBRATION COMPLETE\n");
    Serial.println("---- MAG BIAS VALUES ----\n");
    Serial.print("X_Bias : ");
    Serial.print(hxb);
    Serial.print(',');
    Serial.print(" Y_Bias : ");
    Serial.print(hyb);
    Serial.print(',');
    Serial.print(" Z_Bias : ");
    Serial.println(hzb);

    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("MAG X BIAS : ");
    oled.setCursor(100, 0);
    oled.println(hxb);
    oled.setCursor(0, 25);
    oled.println("MAG Y BIAS : ");
    oled.setCursor(100, 25);
    oled.println(hyb);
    oled.setCursor(0, 50);
    oled.println("MAG Z BIAS : ");
    oled.setCursor(100, 50);
    oled.println(hzb);

    delay(2000);

    Serial.println("\n---- MAG SCALE VALUES ----\n");
    Serial.print("X_Scale : ");
    Serial.print(hxs);
    Serial.print(',');
    Serial.print("Y_Scale : ");
    Serial.print(hys);
    Serial.print(',');
    Serial.print("Z_Scale : ");
    Serial.println(hzs);

    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("MAG X SCALE : ");
    oled.setCursor(100, 0);
    oled.println(hxs);
    oled.setCursor(0, 25);
    oled.println("MAG Y SCALE : ");
    oled.setCursor(100, 25);
    oled.println(hys);
    oled.setCursor(0, 50);
    oled.println("MAG Z SCALE : ");
    oled.setCursor(100, 50);
    oled.println(hzs);

    delay(2000);

    // Set biased/scale factors
    IMU.setMagCalX(hxb, hxs);
    IMU.setMagCalY(hyb, hys);
    IMU.setMagCalZ(hzb, hzs);

    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("MAG BIAS/SCALE VALUES");
    oled.setCursor(30, 25);
    oled.println("ARE STORED");
    oled.display();

    Serial.println("\n---- MAGNOMETER BIAS/SCALE FACTOR SET TO ----\n");

    Serial.print("Mag X Bias : ");
    Serial.print(hxb);
    Serial.print("\tMag X Scale Factor : ");
    Serial.println(hxs);
    Serial.print("Mag Y Bias : ");
    Serial.print(hyb);
    Serial.print("\tMag Y Scale Factor : ");
    Serial.println(hys);
    Serial.print("Mag Z Bias : ");
    Serial.print(hzb);
    Serial.print("\tMag Z Scale Factor : ");
    Serial.println(hzs);

    delay(2000);
  }

  //---------------------------------------------------- [ CALIBRATION STUFF DONE ] ----------------------------------------------------//
  Serial.println("\nPress [ ENTER ] to display IMU data...");
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("PRESS ENTER");
  oled.setCursor(0, 20);
  oled.println("TO DISPLAY");
  oled.setCursor(0, 40);
  oled.println("IMU DATA");

  // Calibration won't start until user inputs 'ENTER' in the serial monitor
  while (true)
  {

    // Checks serial monitor for incoming data
    if (Serial.available() > 0)
    {
      // If user presses 'ENTER' break out of while loop
      if (Serial.read() == '\n')
      {
        break;
      }
    }
    oled.setCursor(60, 50);
    oled.print(".");
    Serial.print(".");
    oled.display();
    delay(100);
  }
  // Uncomment if you're exporting the IMU data to something like Excel
  // Serial.println("ax,ay,az,gx,gy,gz");
}

void loop()
{
  // Reads IMU data from sensor
  IMU.readSensor();
  ax = (IMU.getAccelX_mss());
  ay = (IMU.getAccelY_mss());
  az = (IMU.getAccelZ_mss());
  gx = (IMU.getGyroX_rads());
  gy = (IMU.getGyroY_rads());
  gz = (IMU.getGyroZ_rads());
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();

  // Display data
  // display_values_table();
  display_values_serial();
  delay(100);
}
