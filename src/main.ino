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

long lowerThreshold = -50000L;
long upperThreshold = 50000L;
long center = -29517;

int counter = 0;
long val;

const int num_register = 8;
uint8_t inval[num_register];

HX711 scale;
AccelStepper stepper = AccelStepper(AccelStepper::HALF4WIRE,stepperPhaseA,stepperPhaseC,stepperPhaseB,stepperPhaseD) ;

int scaleToStepsDivider = 200;
long destinationPosition = 0;
long maxTargetPositionDistance = 100;

void setup(){
  scale.begin(loadCellDOutPin,loadCellSckPin,64);
  Serial.begin(115200);
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(1000);
  center = scale.read_average(4);
  //stepper.setSpeed(1000);
}

void loop(){

  if(scale.is_ready()){
    //Serial.print('#');
    long rawValue = scale.read();
    val = distanceFromCenter( rawValue );
    
    if(val<minVal){
      minVal = val;
    } else if(val>maxVal) {
      maxVal = val;
    }

    
    if(Serial.available()){
      char command = Serial.read();
      if(command=='c'){
        center = rawValue;
        Serial.println("");
        Serial.print("calibrated to: ");
        Serial.print(rawValue);
        Serial.println("");
      }
      if(command=='z'){
        stepper.setCurrentPosition(0);
        Serial.println("");
        Serial.println("zeroed position ");
      }
      if(command=='-'){
        scaleToStepsDivider--;
        Serial.println("");
        Serial.print("scaleToStepsDivider to: ");
        Serial.print(scaleToStepsDivider);
        Serial.println("");
      }
      if(command=='+'){
        scaleToStepsDivider++;
        Serial.println("");
        Serial.print("scaleToStepsDivider to: ");
        Serial.print(scaleToStepsDivider);
        Serial.println("");
      }
    }

    long targetPos = stepper.targetPosition();
    if(val>upperThreshold){
    //stepper.setSpeed(-1000);

      long move = (val-upperThreshold)/scaleToStepsDivider;
      if(move>maxTargetPositionDistance){
        move = maxTargetPositionDistance;
      }

      stepper.moveTo(targetPos + move );    
    }
    else if(val<lowerThreshold){
      //stepper.setSpeed(1000);maxTargetPositionDistance

      long move = (val-lowerThreshold)/scaleToStepsDivider;
      if(move<-maxTargetPositionDistance){
        move = -maxTargetPositionDistance;
      }

      stepper.moveTo( targetPos + move );      
    } else {
      //stepper.setSpeed(0);
      stepper.moveTo( destinationPosition );
    }
  }



  
  
  stepper.run();
  //stepper.runSpeed();
}

long distanceFromCenter(long val){
  return val - center;
}

