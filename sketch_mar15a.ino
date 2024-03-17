#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <string.h>

SoftwareSerial gsmSerial(4,5); 
SoftwareSerial mySerial(10, 11); // RX, TX
TinyGPS gps;


void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);


const int ENA = 9;
int speed; 
const int resetButtonPin = 3; 
bool resetButtonState = HIGH;
bool previousResetButtonState = HIGH;
bool reset_motor = true;
const int irPin = 2; // Replace with your IR sensor pin
const int buzzerpin = 7;
int irState = HIGH;
int previousIrState = HIGH;
int irCount = 0;
bool irCountIncreased = false; // Flag to track if count is increased
bool sendGPS = false;
unsigned long irStartTime = 0;
String outputString;
// String buff;



void setup() {
  Serial.begin(9600);
  gsmSerial.begin(9600);
  mySerial.begin(9600);
  
  pinMode(irPin, INPUT);
  pinMode(buzzerpin, OUTPUT);
  pinMode(resetButtonPin, INPUT_PULLUP);
  pinMode(ENA, OUTPUT);
  analogWrite(ENA, 200);

   delay(3000); 

  // Set up GSM module
  gsmSerial.println("AT");
  delay(1000);
  gsmSerial.println("AT+CMGF=1"); 
  delay(1000);

  Serial.println("Setup complete.");
  
}

void loop() {
  // Read IR sensor state
  irState = digitalRead(irPin);
  String command = Serial.readStringUntil('\n');

  // Check if IR sensor state changed to LOW
  
  if (irState == LOW && previousIrState == HIGH) {
    irStartTime = millis(); 
    irCountIncreased = false;
  }

   // Check if IR sensor has been low for at least 2 seconds
  if (irState == LOW && millis() - irStartTime >= 800 && !irCountIncreased) {
    irCount++;
    irCountIncreased = true;
    Serial.print("IR Count: ");
    Serial.println(irCount);
    digitalWrite(buzzerpin, HIGH);
    if (reset_motor) {
    for (speed = 200; speed>=130; speed--) {
      analogWrite(ENA, speed);
      delay(10);
    }
    }
    
  }
  
  // Check if IR sensor state changed to HIGH
  if (irState == HIGH && previousIrState == LOW) {
    irStartTime = 0; // Reset start time when IR sensor goes high
    digitalWrite(buzzerpin, LOW);
     if (reset_motor) {
    for (speed = 130; speed<=200; speed++) {
      analogWrite(ENA, speed);
      delay(10);
    }
    }
  }

  resetButtonState = digitalRead(resetButtonPin);
  if (resetButtonState == LOW && previousResetButtonState == HIGH) {
    irCount = 0; 
    Serial.println("IR Count Reset");
    reset_motor =true;
    analogWrite(ENA, 200);
  }
  previousResetButtonState = resetButtonState;

  if (irCount >=3) {
  reset_motor =false;
  analogWrite(ENA, 0);
  }
  // Check if IR count is 3
  if (irCount == 3) {
    sendGPS = true;
    irCount++;
    Serial.println("Sending GPS data...");
  }

  if (command.equals("send")) {
  sendGPS = true;
  }

  // Check if GPS data needs to be sent
  if (sendGPS) {
     bool newdata = false;
        while (1) {
          if (mySerial.available()) {
            char c = mySerial.read();
            //  Serial.print(c);  // uncomment to see raw GPS data
            if (gps.encode(c)) {
              newdata = true;
              break;  // uncomment to print new data immediately!
            }
          }
        }
        if (newdata) {
          // Serial.println("Acquired Data");
          Serial.println();
          gpsdump(gps);
          Serial.println("");
          sendSMS();
          delay(300);
          // Serial.println("-------------");
          sendSMS();
          delay(300);
          sendSMS();
          Serial.println();
          
        }
     
    
    sendGPS = false; // Reset sendGPS flag
  }

  previousIrState = irState;



  if (irCount>=3) {
   if (Serial.available() > 0) {
    // char c = Serial.read();
    if (irCount>=3) {
      sendSMS();
    }
  }
  }


} 



void gpsdump(TinyGPS &gps)
{
  // long lat, lon;
  float flat, flon;
  // unsigned long age;
  // unsigned short sentences, failed;

  // gps.get_position(&lat, &lon);
  // Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
 

  gps.f_get_position(&flat, &flon);
  // Serial.print("https://maps.google.com/maps?q=loc:"); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  outputString = "";

  outputString += "https://maps.google.com/maps?q=loc:";
  outputString += String(flat, 5);
  outputString += ",";
  outputString += String(flon, 5);
  Serial.print(outputString);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

void sendSMS() {
  Serial.println("Sending SMS...");

  
  gsmSerial.println("AT+CMGS=\"+918955393300\""); 
  delay(1000);
  gsmSerial.print(outputString); 
  delay(1000);
  gsmSerial.println((char)26); 
  delay(1000);

  Serial.println("SMS sent.");
}
