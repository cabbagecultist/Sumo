#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

enum State
{
  Scanning,            // Robot scans from static position.  
  Wander,              // Robot wanders around.  
  Rotate2Target,       // Assumes target is found, will rotate to largest prox value
  ChargeTarget,        // Charges a target. 
  EdgeRecovery         // Recovers from finding an edge.
};

bool lineSensorValues[5];   // line sensor detection.  Array of [left, middle left, middle, middle right, right]
State currentState = Scanning;   //We start by scanning, but honestly could switch to wander if want to

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);

  
}

void loop() {
  // put your main code here, to run repeatedly:

  switch (currentState) {
    case Scanning:
      {
        
        if() { //if sensors detect OPPONENTNTTNTNTTNTNTNTTNT  ps: tnt = big boom
          currentState=Rotate2Target;
        } else { 
          currentState=Wander;
        }
      }
      break;

   case Rotate2Target:
      {
        //add code to turn menacingly towards the opponent robot  
        currentState=ChargeTarget;
      }
      break;
    case ChargeTarget:
      {
        // add code for ATTACKING a target
        currentState=EdgeRecovery;
      }
      break;

    case EdgeRecovery:
      {
        // add code for recovering from hitting the edge of the field
        currentState=Scanning;
      }
      break;

    case Wander:
      {
        // add code for wander around like a predator searching for his prey :)
        if() { //here if one of the floor sensors detect the while line edge thingy
          currentState=stateEdgeRecovery;
        } else if() { //here if one of the tof sensors see the opponent robottt
          currentState=stateRotate2Target;
        } 
      }
      break;
  } // This ends the switch case block
} 



// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}