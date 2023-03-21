int setPointTrim = A0;
int pTrim = A1;
int iTrim = A2;
int dTrim = A3;

int setPointTrimValue = 0;
int pTrimValue = 0;
int iTrimValue = 0;
int dTrimValue = 0;

void setup() {
  Serial.begin(115200);

  
}

void loop() {
  
  setPointTrimValue = analogRead(setPointTrim);
  pTrimValue = analogRead(pTrim);
  iTrimValue = analogRead(iTrim);
  dTrimValue = analogRead(dTrim);

  Serial.print("setPointTrimValue: ");
        
}
