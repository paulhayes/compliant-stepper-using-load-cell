#include <HX711.h>
#include <AccelStepper.h>

const int loadCellDOutPin = 2;
const int loadCellSckPin = 3;
const int stepperPhaseA = 4;
const int stepperPhaseB = 5;
const int stepperPhaseC = 6;
const int stepperPhaseD = 7;

long maxVal;
long minVal;

long lowerThreshold = -200000L;
long upperThreshold = 200000L;
long center = -50000L;

int counter = 0;
long val;

const int num_register = 8;
uint8_t inval[num_register];

HX711 scale;
AccelStepper stepper = AccelStepper(AccelStepper::HALF4WIRE,stepperPhaseA,stepperPhaseC,stepperPhaseB,stepperPhaseD) ;


void setup(){
  scale.begin(loadCellDOutPin,loadCellSckPin,64);
  Serial.begin(115200);
  stepper.setMaxSpeed(10000);
  stepper.setSpeed(1000);
}

void loop(){

  if(scale.is_ready()){
    val = distanceFromCenter( scale.read() );
    
    if(val<minVal){
      minVal = val;
      lowerThreshold = val / 10;
    } else if(val>maxVal) {
      maxVal = val;
      upperThreshold = val / 10;
    }
  }


  if(val>upperThreshold){
    stepper.setSpeed(-1000);
    
  }
  else if(val<lowerThreshold){
    stepper.setSpeed(1000);
    
  } else {
    stepper.setSpeed(0);
  }
  
  
  stepper.runSpeed();
}

long distanceFromCenter(long val){
  return val - center;
}

