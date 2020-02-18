void setup() {
  // put your setup code here, to run once:
// Start serial communication at 115200 baud
  Serial.begin(115200);
  
  // Give the HMC6343 a half second to wake up
  delay(500); 
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println("Heading Data (Raw value, in degrees):");

//  char inByte = ' ';
//  if(Serial.available()){ // only send data back if data has been sent
//  char inByte = Serial.read(); // read the incoming data
  Serial.print("from arduino test");
//  Serial.print(inByte); // send the data back in a new line so that it is not all one long line
  Serial.println();
//  }

  delay(500);
}
