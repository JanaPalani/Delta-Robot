#include <string.h>
void setup() {
  Serial.begin(9600);
}

void loop() {
  static char buffer[32]; // Buffer to store received characters
  static int bufferIndex = 0; // Index to keep track of buffer position
  char *token;
  int x,y,z ;

  if (Serial.available()) {
    char receivedChar = Serial.read(); // Read the incoming character

    // Check if the character is a digit or newline
    if (isdigit(receivedChar) || receivedChar == '\n' || receivedChar == ',') {
      buffer[bufferIndex++] = receivedChar;

      // If newline is received, convert the buffer to an integer and print
      if (receivedChar == '\n') {
        buffer[bufferIndex] = '\0'; // Null-terminate the string
        // int receivedNumber = atoi(buffer);
         token  = strtok(buffer,",");
         x = atoi(token);
         token  = strtok(NULL,",");
         y = atoi(token);
         token  = strtok(NULL,",");
         z = atoi(token);
         Serial.print(x);
         Serial.print(",");
         Serial.print(y);
         Serial.print(",");
         Serial.println(z);
         delay(30000);
         Serial.println("one point completed");
        
         bufferIndex = 0;
      }
    }
  }
}







