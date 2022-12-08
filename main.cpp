
#include "mbed.h"
#include "uLCD_4DGL.h"
#include "rtos.h"
#include "PinDetect.h"
#include "BME280.h"
#include "wave_player.h"
#include "cylonbyc.h"
#define sample_freq 11025.0

#define MAX_TIME 10 // Time in seconds. Max number of samples held in arrays

//get and set the frequency from wav conversion tool GUI
volatile int ii=0;

//Class to control an RGB LED using three PWM pins
class RGBLed
{
public:
    RGBLed(PinName redpin, PinName greenpin, PinName bluepin);
    void write(float red,float green, float blue);
private:
    PwmOut _redpin;
    PwmOut _greenpin;
    PwmOut _bluepin;
};
 
RGBLed::RGBLed (PinName redpin, PinName greenpin, PinName bluepin)
    : _redpin(redpin), _greenpin(greenpin), _bluepin(bluepin)
{
    //50Hz PWM clock default a bit too low, go to 2000Hz (less flicker)
    _redpin.period(0.0005);
}
 
void RGBLed::write(float red,float green, float blue)
{
    _redpin = red;
    _greenpin = green;
    _bluepin = blue;
}
//class could be moved to include file
 
 
//Setup RGB led using PWM pins and class
RGBLed myRGBled(p23,p22,p21); //RGB PWM pins
//SDFileSystem sd(p5, p6, p7, p8, "sd");
RawSerial pc(USBTX, USBRX);
uLCD_4DGL uLCD(p13,p14,p11); // serial tx, serial rx, reset pin;
DigitalOut led(LED1); //heartbeat led
DigitalOut led1(LED2);
DigitalOut led4(LED3);
//PwmOut speaker (p24); // for freeze alarm
AnalogOut speaker (p18);
PinDetect pb1(p19, PullUp); //debounced pushbutton using interrupts
PinDetect pb2(p20, PullUp); //debounced pushbutton using interrupts
RawSerial blue(p9,p10);
BME280 sensor(p28, p27);
Ticker tick;
Ticker sample_tick;
//AnalogOut DACout(p18);
//wave_player waver(&DACout);

DigitalOut Ctrl(p16);

Mutex LCD;
volatile float dtemp = 20.0;
volatile float curr_temp =0.0;
volatile float humidity =0.0;
volatile float pressure =0.0;

volatile float curr_temp_arr[MAX_TIME];
volatile float humidity_arr[MAX_TIME];
volatile float pressure_arr[MAX_TIME];

volatile float curr_temp_avg;
volatile float humidity_avg;
volatile float pressure_avg;

volatile int i = 0;
volatile int j = 0;
volatile bool isAverage = false;
volatile bool isCelsius = false;
volatile bool isSetting = false;
volatile bool was_dtemp_higher_than_currtemp = false;
volatile bool was_setmode_on = false;
volatile char bnum = 0;
volatile char bhit = 0;
volatile char vara;
volatile char varb;
volatile char varc;
volatile char vard;

void pb_hit (void)
{
    isCelsius = !isCelsius;
}

void pb_hit2 (void)
{
    isAverage = !isAverage;
}

//interrupt routine to play next audio sample from array in flash

void heating_on_audio ()
{
    speaker.write_u16(sound_data[ii]);
    ii++;
    if (ii>= NUM_ELEMENTS) {
        ii = 0;
        sample_tick.detach();
    }
}

void cooling_on_audio ()
{
    speaker.write_u16(sound_data[ii]);
    ii++;
    if (ii>= NUM_ELEMENTS) {
        ii = 0;
        sample_tick.detach();
    }
}

void tock (void)
{
    curr_temp_arr[i] = curr_temp;
    humidity_arr[i] = humidity;
    pressure_arr[i] = pressure;
    curr_temp_avg = 0;
    humidity_avg = 0;
    pressure_avg = 0;
    for (j = 0; j < MAX_TIME; j++) {
        curr_temp_avg = curr_temp_avg + curr_temp_arr[j];
        humidity_avg = humidity_avg + humidity_arr[j];
        pressure_avg = pressure_avg + pressure_arr[j];
    }
    curr_temp_avg = curr_temp_avg / MAX_TIME;
    humidity_avg = humidity_avg / MAX_TIME;
    pressure_avg = pressure_avg / MAX_TIME;
    
    i++;
    if(i == MAX_TIME) {
        i = 0;
    }
    
}

void Display_Desired_Temp(void const*argument)
{
    while(1)
    {
        LCD.lock();
        uLCD.color(BLUE);
        uLCD.text_height(1);
        uLCD.locate(0,2);
        uLCD.printf("Desired Temp:");
        uLCD.locate(4,3);
        if(isCelsius) {
        uLCD.printf("%2.2f degC", dtemp);
        } else {
        uLCD.printf("%2.2f degF", (dtemp*1.8+32));
        }
        LCD.unlock();
        Thread::yield();
        Thread::wait(1000);
    }
}

void mbedLED(void const*argument)
{
    while(1)
    {
        led = !led;
        Thread::yield();
        Thread::wait(1000);
    }
}

void ControlCommands(void const*argument)
{
    while (1)
    {
        if(!blue.readable()) Thread::yield();
        if(blue.readable())
        {
            LCD.lock();
            //pc.printf("Readable\r\n");
            //pc.printf("%c\r\n", blue.getc());
            if(blue.readable()){
                vara = blue.getc();
                }
            if(blue.readable()){
                varb = blue.getc();
                }
            if(blue.readable()){
                varc = blue.getc();
                }
            //vara = blue.getc();
            //varb = blue.getc();
            //varc = blue.getc();
            //vard = blue.getc();
            pc.printf("%c%c%c", vara, varb, varc);
            //uLCD.locate(0,12);
            //uLCD.printf("%c%c%c", vara, varb, varc);
            if (vara == 'D' && varb == '+' && varc == '+') {
                if(isCelsius) {
                    dtemp = dtemp + 1;
                } else {
                    dtemp = dtemp + 0.5555;
                }
                }
            if (vara == 'D' && varb == '-' && varc == '-') {
                if(isCelsius) {
                    dtemp = dtemp - 1;
                } else {
                    dtemp = dtemp - 0.5555;
                }
                }
            if (vara == 'S' && varb == 'E' && varc == 'T') {
                isSetting = !isSetting;
                }
            if (vara == '+' && isSetting) {
                if(isCelsius) {
                    curr_temp = 10*(varb-48)+1*(varc-48);
                } else {
                    curr_temp = ((10*(varb-48)+1*(varc-48))-32)*0.5555;
                }
                }
                
            if (vara == '-' && isSetting) {
                if(isCelsius) {
                    curr_temp = -(10*(varb-48)+1*(varc-48));
                } else {
                    curr_temp = (-(10*(varb-48)+1*(varc-48))-32)*0.5555;
                }
                }                            
            LCD.unlock();
            //Thread::wait(100.0);
        }
        Thread::yield();
        Thread::wait(100.0);
    //Thread::yield();
    }
}



int main()
{
    Ctrl = 0;
    LCD.lock();
    uLCD.text_height(1);
    uLCD.text_width(1);
    uLCD.locate(4,0);
    uLCD.printf("THERMOSTAT");
    //speaker.period(1.0/800);
    pb1.attach_asserted(&pb_hit);
    pb1.setSampleFrequency();
    pb2.attach_asserted(&pb_hit2);
    pb2.setSampleFrequency();
    LCD.unlock();

    Thread thread2(Display_Desired_Temp); //can combine max temp and min temp to one thread to have lesser threads in total????
    Thread thread3(mbedLED);
    Thread thread4(ControlCommands);

    tick.attach(&tock, 1.0);
    
    LCD.lock();
    pc.printf("TestHello\r\n");
    LCD.unlock();
    
    
    curr_temp = sensor.getTemperature();
    if(dtemp > curr_temp)
    {
        was_dtemp_higher_than_currtemp = true;    
    }
    else
    {
        was_dtemp_higher_than_currtemp = false;                
    }
                
                
    while(1)
    {
        if(!isSetting)
        {
            curr_temp = sensor.getTemperature();
        }
        humidity = sensor.getHumidity();
        pressure = sensor.getPressure();
        
        //dtemp = sensor.getTemperature();
        LCD.lock();
        
        if(isAverage)
        {
            uLCD.locate(0,4);
            uLCD.color(GREEN);
            uLCD.text_height(1);
            uLCD.printf("Avg Temp:     ");
            uLCD.locate(4,5);        
            uLCD.text_height(2);
            uLCD.text_width(1);
            if(isCelsius)
            {
                uLCD.printf("%2.2f degC", curr_temp_avg);
            } else
            {
                uLCD.printf("%2.2f degF", (curr_temp_avg*1.8+32));
            }
            uLCD.color(WHITE);
            uLCD.text_height(1);
            uLCD.text_width(1);
            uLCD.locate(0,8);
            uLCD.printf("Avg Humidity:");
            uLCD.locate(4,9);
            uLCD.printf("%2.2f %%", humidity_avg);
            uLCD.locate(0,10);
            uLCD.printf("Avg Pressure:");
            uLCD.locate(4,11);
            uLCD.printf("%04.2f hPa", pressure_avg);            
        }
        else
        {
            uLCD.locate(0,4);
            uLCD.color(GREEN);
            uLCD.text_height(1);
            uLCD.printf("Current Temp:");
            uLCD.locate(4,5);        
            uLCD.text_height(2);
            uLCD.text_width(1);
            if(isCelsius)
            {
                uLCD.printf("%2.2f degC", curr_temp);
            }
            else
            {
                uLCD.printf("%2.2f degF", (curr_temp*1.8+32));
            }
            uLCD.color(WHITE);
            uLCD.text_height(1);
            uLCD.text_width(1);
            uLCD.locate(0,8);
            uLCD.printf("Humidity:     ");
            uLCD.locate(4,9);
            uLCD.printf("%2.2f %%", humidity);
            uLCD.locate(0,10);
            uLCD.printf("Pressure:     ");
            uLCD.locate(4,11);
            uLCD.printf("%04.2f hPa", pressure);
        }
        if(dtemp > curr_temp + 1)
        {
            uLCD.color(RED);
            uLCD.text_height(1);
            uLCD.locate(4,13);
            uLCD.printf("HEATING");
            myRGBled.write(1.0,0.0,0.0);
            Ctrl=0;
            //sample_tick.attach(&heating_on_audio, 1.0 / sample_freq);
            Thread::wait(500);
            if(was_dtemp_higher_than_currtemp == false)
            {
               sample_tick.attach(&heating_on_audio, 1.0 / sample_freq);
               // speaker.period(1.0/500.0); // 500hz period
               // speaker =0.25; //50% duty cycle - max volume
                Thread::wait(4000);
               // speaker=0.0; // turn off audio
                was_dtemp_higher_than_currtemp = true;                
            }
        }
        else if (dtemp  < curr_temp - 1)
        {
            uLCD.color(BLUE);
            uLCD.text_height(1);
            uLCD.locate(4,13);
            uLCD.printf("COOLING");
            myRGBled.write(0.0,0.0,1.0);
            Ctrl=1;
            //sample_tick.attach(&cooling_on_audio, 1.0 / sample_freq);
            Thread::wait(500);
            if(was_dtemp_higher_than_currtemp == true)
            {
                sample_tick.attach(&cooling_on_audio, 1.0 / sample_freq);
                //speaker.period(1.0/400.0); // 400hz period
                //speaker =0.25; //50% duty cycle - max volume
                Thread::wait(4000);
            // speaker=0.0; // turn off audio
                was_dtemp_higher_than_currtemp = false;
            }
        }
        else
        {
            uLCD.color(BLUE);
            uLCD.text_height(1);
            uLCD.locate(4,13);
            uLCD.printf("        ");
            myRGBled.write(0.0,0.0,0.0);
            Ctrl=0;
        }
        if(isSetting)
        {
            uLCD.color(GREEN);
            uLCD.text_height(1);
            uLCD.locate(2,14);
            uLCD.printf("SET TEMP MODE");
            if(was_setmode_on == false)
            {
                // speaker.period(1.0/600.0); // 600hz period
                //speaker =0.25; //50% duty cycle - max volume
                Thread::wait(500);
                //speaker=0.0; // turn off audio
                was_setmode_on = true;            
            }
        }
        else
        {
            uLCD.color(GREEN);
            uLCD.text_height(1);
            uLCD.locate(2,14);
            uLCD.printf("              ");
            if(was_setmode_on == true)
            {
                // speaker.period(1.0/700.0); // 700hz period
                //speaker =0.25; //50% duty cycle - max volume
                Thread::wait(500);
                //speaker=0.0; // turn off audio   
                was_setmode_on = false;
            }
        }           
        LCD.unlock();
        Thread::yield();
        Thread::wait(500.0);
    }
}
