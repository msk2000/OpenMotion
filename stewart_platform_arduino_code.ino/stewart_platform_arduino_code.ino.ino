#include <Servo.h>

#define NUM_SERVOS 6
#define LEVEL_POS 90

// array of Servo objects
Servo servos[NUM_SERVOS];

// Variables to store the angles for each servo
int servoAngles[NUM_SERVOS] = {LEVEL_POS,LEVEL_POS,LEVEL_POS,LEVEL_POS,LEVEL_POS,LEVEL_POS};

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    
    // Attach servos to the corresponding pins
    servos[0].attach(3);  // Servo 0 connected to pin 3
    servos[1].attach(5);  // Servo 1 connected to pin 5
    servos[2].attach(6);  // Servo 2 connected to pin 6
    servos[3].attach(9);  // Servo 3 connected to pin 9
    servos[4].attach(10); // Servo 4 connected to pin 10
    servos[5].attach(11); // Servo 5 connected to pin 11

   
  // Set all servos to level position (45 degrees)
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(servoAngles[i]);
    }
}

void loop() 
{
    // Check if data is available to read
    if (Serial.available() > 0) 
    {
        // Read the incoming data as a string (excluding nL)
        String input = Serial.readStringUntil('\n');

        // Parse the input data
        parseInput(input);
        
        // Set servo angles
        for (int i = 0; i < NUM_SERVOS; i++) 
        {
          if (i % 2 == 0)
          {
            servos[i].write(servoAngles[i]); // Set the angle for servos 0,2,4
          }
          else 
          {
          servos[i].write(180-servoAngles[i]); // Set the angle for servos 1,3,5
            
          }
        }
    }
}

// Function to parse the input string and set servo angles
void parseInput(String input) {
    // Split the input string by commas
    int startIndex = 0; //keeps track of the starting position for substring extraction
    int commaIndex = input.indexOf(',', startIndex); //finds the index of the first comma
    int servoIndex = 0; //counts how many servo angles have been parsed

  //loop continues until there are no more commas
    while (commaIndex != -1 && servoIndex < NUM_SERVOS) {
        // Extract the angle and convert to integer
        String angleString = input.substring(startIndex, commaIndex); // one parsed string(angle)
        servoAngles[servoIndex] = angleString.toInt(); // conversion to int
        
        // Move to the next servo index and update the startIndex
        servoIndex++;
        startIndex = commaIndex + 1;//point to the position right after the comma
        commaIndex = input.indexOf(',', startIndex);//searches for the next comma in the updated substring 
    }

    // Handle the last angle (after the last comma)
    if (servoIndex < NUM_SERVOS) {
        String angleString = input.substring(startIndex);
        servoAngles[servoIndex] = angleString.toInt();
    }
}
