//#include <HX711.h>
#define PLOT_MODE false

#include <Arduino.h>
#include <AccelStepper.h>
#include <ADC.h>
#include <ADC_util.h>
#include "Plotter.h"
#include <SimpleKalmanFilter.h>

const int loadCellDOutPin = 2;
const int loadCellSckPin = 3;
const int stepperPhaseA = 4;
const int stepperPhaseB = 5;
const int stepperPhaseC = 6;
const int stepperPhaseD = 7;

long maxVal;
long minVal;

double compliantForceThreshold = 1;
double returnThreshold = 0.5;
double center = 0;

int counter = 0;
double val = 0;
double lowPassFilteredValue = 0;
double sKalmanFilteredValue = 0;

/*
 This sample code demonstrates how to use the SimpleKalmanFilter object. 
 Use a potentiometer in Analog input A0 as a source for the reference real value.
 Some random noise will be generated over this value and used as a measured value.
 The estimated value obtained from SimpleKalmanFilter should match the real
 reference value.

 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.1);

const int num_register = 8;
uint8_t inval[num_register];

//HX711 scale;
AccelStepper stepper = AccelStepper(AccelStepper::HALF4WIRE,stepperPhaseA,stepperPhaseC,stepperPhaseB,stepperPhaseD) ;
Plotter p;

int scaleToSteps = 300;
long destinationPosition = 0;
long maxTargetPositionDistance = 100;
const int sendRate = 120;
const int secondsToPlot = 20;
long lastSendMillis;
int filter = 200;
int adcReadCount = 0;
float adcReadRate = 0;
long move;

ADC *adc = new ADC(); // adc object
bool newADCValue = false;

void setup(){
  //scale.begin(loadCellDOutPin,loadCellSckPin,64);
  
  //Serial.begin(115200);
#if PLOT_MODE
  p.Begin();
  p.AddTimeGraph( "Some title of a graph", sendRate*10, "kalman filter", sKalmanFilteredValue, "low pass filter",lowPassFilteredValue, "centered", val, "move", move, "adc read rate khz", adcReadRate );
#else
  Serial.begin(115200);
#endif
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(2000);
  //center = scale.read_average(4);
  //stepper.setSpeed(1000);
  adc->adc1->setAveraging(32); // set number of averages
  adc->adc1->setResolution(16); // set bits of resolution
  // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  // see the documentation for more information
  // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED); // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED); // change the sampling speed
  
  
  adc->adc1->enableInterrupts(adc1_isr);
  adc->startContinuousDifferential(A10,A11,ADC_1);
  //adc->adc1->startContinuousDifferential(10,11);

  attachInterrupt(8,onButtonUp,RISING);
}

void onButtonUp(){
  center = sKalmanFilteredValue;
}

void loop(){
#if not PLOT_MODE
  if(adc->adc1->fail_flag != ADC_ERROR::CLEAR) {
    Serial.print("Error ADC0: "); Serial.println(getStringADCError(adc->adc1->fail_flag));
  }
#endif

  if(newADCValue){
    newADCValue = false;
    int rawValue = -adc->adc1->analogReadContinuous();
    adcReadCount++;
    lowPassFilteredValue = (rawValue+lowPassFilteredValue*filter) / (filter+1);
    sKalmanFilteredValue = simpleKalmanFilter.updateEstimate(rawValue);
    
    val = distanceFromCenter( sKalmanFilteredValue );
    
#if PLOT_MODE
    long currentTime = millis();
    int sendDuration = 1000/sendRate;
    if(sendDuration<=currentTime-lastSendMillis){
      
      adcReadRate = 1.0f*adcReadCount / (currentTime-lastSendMillis);
  
      p.Plot();
      lastSendMillis = currentTime;
      adcReadCount = 0;
    }
#endif
    //Serial.println(val);


    if(val<minVal){
      minVal = val;
    } else if(val>maxVal) {
      maxVal = val;
    }

#if not PLOT_MODE
    if(Serial.available()){
      char command = Serial.read();
      if(command=='z'){
        stepper.setCurrentPosition(0);
        Serial.println("");
        Serial.println("zeroed position ");
      }
      if(command=='-'){
        scaleToSteps--;
        Serial.println("");
        Serial.print("scaleToSteps to: ");
        Serial.print(scaleToSteps);
        Serial.println("");
      }
      if(command=='+'){
        scaleToSteps++;
        Serial.println("");
        Serial.print("threshold to: ");
        Serial.print(compliantForceThreshold);
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
        Serial.print("scaleToSteps to: ");
        Serial.print(compliantForceThreshold);
        Serial.println("");
      }
    }
#endif
    long targetPos = stepper.targetPosition();
    long currentPos = stepper.currentPosition();
    if(val>compliantForceThreshold){
    //stepper.setSpeed(-1000);

      move = (int)round(val)*scaleToSteps;
      
      stepper.setSpeed( move );
      stepper.runSpeed();
      //stepper.moveTo(currentPos + move );    
    }
    else if(val<-compliantForceThreshold){

      move = (int)round(val)*scaleToSteps;
    
      //stepper.moveTo( currentPos + move );
      stepper.setSpeed( move );
      stepper.runSpeed();
    } else if(abs(val)<returnThreshold){
      move = 0;
      stepper.moveTo( destinationPosition );
      stepper.run();
    }

    
  }

  
}

double distanceFromCenter(double val){
  return val - center;
}

void adc1_isr(void) {
    adc->adc1->analogReadContinuous();
    newADCValue = true;
}