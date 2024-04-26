#include <DFRobot_BMP3XX.h>
#include <LiquidCrystal_I2C.h>
// DFRobot_BMP388_I2C sensor_internal;
// DFRobot_BMP388_I2C sensor_bottom;

LiquidCrystal_I2C lcd(0x27,20,4); // 0x27
DFRobot_BMP388_I2C sensor_internal(&Wire, sensor_internal.eSDOVDD); // 0x76
DFRobot_BMP388_I2C sensor_bottom(&Wire, sensor_bottom.eSDOGND); // 0x77


#define CALIBRATE_ABSOLUTE_DIFFERENCE
int boot_t;
bool finalText = 1;
float F_temp_internal;
float F_temp_bottom;

void setup()
{
    initialLCD();
    initial_barometers();
    printAuthor();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Internal  |  Bottom ");

    boot_t = millis();
}

void loop()
{
    /* Read currently measured temperature date directly, unit: °C */
    float temperature_internal = sensor_internal.readTempC();
    float temperature_bottom = sensor_bottom.readTempC();
    /* Directly read the currently measured pressure data, unit: pa */
    float Pressure_internal = sensor_internal.readPressPa();
    float Pressure_bottom = sensor_bottom.readPressPa();
    
    lcd.setCursor(0,1);
    lcd.print("T : ");
    lcd.print(temperature_internal);
    lcd.print(" |  ");
    lcd.print(temperature_bottom);
    lcd.print(" ");

    lcd.setCursor(0,2);
    lcd.print("P :");
    lcd.print(long(Pressure_internal));
    lcd.print(" | ");
    lcd.print(long(Pressure_bottom));

    if(finalText && (millis() - boot_t) > 30000)
    {
        F_temp_internal = temperature_internal;
        F_temp_bottom = temperature_bottom;
        finalText = 0;

        lcd.setCursor(0,3);
        lcd.print("Tf: ");
        lcd.print(F_temp_internal);
        lcd.print(" |  ");
        lcd.print(F_temp_bottom);
        lcd.print(" ");
    }

    delay(500);
    // lcd.clear();
}


int initial_barometers(void)
{
    int rslt;
    //Internal sensor initiate
    while( ERR_OK != (rslt = sensor_internal.begin()) )
    {
        if(ERR_DATA_BUS == rslt)
        {
            lcd.println("Data bus error!!!");
        }
        else if(ERR_IC_VERSION == rslt)
        {
            lcd.println("Chip ver. not match!!!");
        }
        delay(3000);
    }
    //Bottom sensor initiate
    while( ERR_OK != (rslt = sensor_bottom.begin()) )
    {
        if(ERR_DATA_BUS == rslt)
        {
            lcd.println("Data bus error!!!");
        }
        else if(ERR_IC_VERSION == rslt)
        {
            lcd.println("Chip ver. not match!");
        }
        delay(3000);
    }

    lcd.println("Sensors begin : OK");

    //Internal sensor sampling mode checking
    while( !sensor_internal.setSamplingMode(sensor_internal.eUltraPrecision) )
    {
        lcd.println("samp. fail, retry...");
        delay(3000);
    }

    //Bottom sensor sampling mode checking
    while( !sensor_bottom.setSamplingMode(sensor_bottom.eUltraPrecision) )
    {
        lcd.println("samp. fail, retry...");
        delay(3000);
    }

    #ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
    //Calibrate sensor with ground truth altitute. (84m @DGIST-E5, Daegu, Korea)
    if( sensor_internal.calibratedAbsoluteDifference(84.0) && sensor_bottom.calibratedAbsoluteDifference(84.0) )
    {
        lcd.println("Calibration done.");
    }
    #endif

    delay(1000);
    lcd.clear();

    return 0;
}

int initialLCD()
{
    lcd.init();
    lcd.backlight();

    int idx = 0;
    for(;idx<20;idx++)
    {
        lcd.setCursor(idx,0);
        lcd.print("*");
        lcd.setCursor(idx,1);
        lcd.print("*");
        lcd.setCursor(idx,2);
        lcd.print("*");
        lcd.setCursor(idx,3);
        lcd.print("*");
        delay(50);
    }
    delay(500);
    lcd.clear();
    lcd.setCursor(0,0);

    return 0;
}

int printStatus(void)
{
    lcd.setCursor(0,0);
    lcd.print("Internal  |  Bottom ");

    //Get current sensor sampling properies
    lcd.setCursor(0,1);
    int samplingPeriodus_bottom = sensor_bottom.getSamplingPeriodUS();
    int samplingPeriodus_internal = sensor_internal.getSamplingPeriodUS();
    lcd.print("P :");
    lcd.print(samplingPeriodus_internal);
    lcd.print(" |  ");
    lcd.print(samplingPeriodus_bottom);

    /* Get the sampling frequency of the current measurement mode, unit: Hz */
    lcd.setCursor(0,2);
    int samplingFrequencyHz_bottom = 1000000 / samplingPeriodus_bottom;
    int samplingFrequencyHz_internal = 1000000 / samplingPeriodus_internal;
    lcd.print("F :");
    lcd.print(samplingPeriodus_internal);
    lcd.print(" |  ");
    lcd.print(samplingPeriodus_bottom);
}

int printAuthor(void)
{
    lcd.setCursor(0,0);
    lcd.print("Author: Bongsub Song");
    lcd.setCursor(0,2);
    lcd.print("     Description    ");
    lcd.setCursor(0,3);
    lcd.print("After 30sec Tf fixed");

    delay(2000);
}