//#include <HX711.h>
#define PLOT_MODE true

#include <Arduino.h>
#include <AccelStepper.h>
#include <ADC.h>
#include <ADC_util.h>
#include "Plotter.h"
#include <SimpleKalmanFilter.h>

const int stepperPhaseA = 4;
const int stepperPhaseB = 5;
const int stepperPhaseC = 6;
const int stepperPhaseD = 7;

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.1);

AccelStepper stepper = AccelStepper(AccelStepper::HALF4WIRE,stepperPhaseA,stepperPhaseC,stepperPhaseB,stepperPhaseD) ;
Plotter p;

ADC *adc = new ADC(); // adc object


double compliantForceThreshold = 5;
double returnThreshold = 4.5;
double center = 0;

int counter = 0;
int rawValue = 0;
double val = 0;
double lowPassFilteredValue = 0;
double sKalmanFilteredValue = 0;

int inputToSteps = 50;
long destinationPosition = 0;
long maxTargetPositionDistance = 100;
const int sendRate = 120;
const int secondsToPlot = 20;
long lastSendMillis;
long lastSendADCCount = 0;
int filter = 200;
long adcReadCount = 0;
float adcReadRate = 0;
long move;

bool newADCValue = false;
bool isCalibrated = false;

void setup(){

#if PLOT_MODE
  p.Begin();
  p.AddTimeGraph( "Some title of a graph", sendRate*10, "raw input",rawValue, "kalman filter", sKalmanFilteredValue, "low pass filter",lowPassFilteredValue, "centered", val, "threshold", compliantForceThreshold, "adc read rate khz", adcReadRate );
#else
  Serial.begin(115200);
#endif
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(4000);
  
  adc->adc0->setReference(ADC_REFERENCE::REF_1V2);
  // see README.md as to why I don't use this
  //adc->adc0->enablePGA(0);
  adc->adc0->setAveraging(32); // set number of averages
  adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
  
  adc->adc0->enableInterrupts(adc1_isr);
  adc->adc0->startContinuousDifferential(A10,A11);
  
  pinMode(8,INPUT_PULLUP);
  attachInterrupt(8,onButtonUp,RISING);
}


void loop(){

  if(newADCValue){
    newADCValue = false;
    readADC();
    #if PLOT_MODE
    plot();
    #else
    serialDebug();
    #endif
  }

  if(!isCalibrated)
    return;

  moveMotor();
}

void readADC(){
      rawValue = -adc->adc0->analogReadContinuous();
    #if not PLOT_MODE
    if(adc->adc0->fail_flag != ADC_ERROR::CLEAR) {
      Serial.print("Error ADC1: "); Serial.println(getStringADCError(adc->adc0->fail_flag));
    }
    #endif
    adcReadCount++;
    lowPassFilteredValue = (rawValue+lowPassFilteredValue*filter) / (filter+1);
    sKalmanFilteredValue = simpleKalmanFilter.updateEstimate(rawValue);
    
    val = distanceFromCenter( sKalmanFilteredValue );
    if(!isCalibrated && (adcReadCount>3000L)){
      calibrate();
    }
}

void moveMotor(){
  if(val>compliantForceThreshold){    
      move = (int)round(val)*inputToSteps;      
      stepper.setSpeed( move );
      stepper.runSpeed();
    }
    else if(val<-compliantForceThreshold){
      move = (int)round(val)*inputToSteps;    
      stepper.setSpeed( move );
      stepper.runSpeed();
    } else if(abs(val)<returnThreshold){
      move = 0;
      stepper.moveTo( destinationPosition );
      stepper.run();
    }
}

void calibrate(){
  center = lowPassFilteredValue;
  isCalibrated = true;
}

double distanceFromCenter(double val){
  return val - center;
}

void plot(){
    long currentTime = millis();
    int sendDuration = 1000/sendRate;
    if(sendDuration<=currentTime-lastSendMillis){
      
      adcReadRate = 1.0f*( adcReadCount-lastSendADCCount ) / (currentTime-lastSendMillis);
  
      p.Plot();
      lastSendMillis = currentTime;
      lastSendADCCount = adcReadCount;
    }

}

void serialDebug(){

    if(Serial.available()){
      char command = Serial.read();
      if(command=='z'){
        Serial.println("");
        stepper.setCurrentPosition(0);
        Serial.println("");
        Serial.println("zeroed position ");
      }
      if(command=='-'){
        inputToSteps--;
        Serial.println("");
        Serial.print("set inputToSteps to: ");
        Serial.print(inputToSteps);
        Serial.println("");
      }
      if(command=='+'){
        inputToSteps++;
        Serial.println("");
        Serial.print("set inputToSteps to: ");
        Serial.print(inputToSteps);
        Serial.println("");
      }
      if(command=='<'){
        compliantForceThreshold--;
        Serial.println("");
        Serial.print("threshold to: ");
        Serial.print(compliantForceThreshold);
        Serial.println("");
      }
      if(command=='>'){
        compliantForceThreshold++;
        Serial.println("");
        Serial.print("threshold to: ");
        Serial.print(compliantForceThreshold);
        Serial.println("");
      }
      if(command=='v'){
        compliantForceThreshold++;
        
        Serial.println("");
        Serial.print("current value=");
        Serial.print(val);
        Serial.print(" low pass value=");
        Serial.print(lowPassFilteredValue);
        Serial.println("");
      }
      
    }
}

/* Interrupts */

void onButtonUp(){
  if(isCalibrated){
    calibrate();
  }
}

void adc1_isr(void) {
    adc->adc0->analogReadContinuous();
    newADCValue = true;
}