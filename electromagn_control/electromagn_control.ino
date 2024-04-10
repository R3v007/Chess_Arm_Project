 const int crtl_pin=13;
 int value =0;

void setup() {
  pinMode(crtl_pin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    String inp=Serial.readStringUntil('\n');
    value = inp.toInt();
    switch (value){
      case 1:
        digitalWrite(crtl_pin, HIGH);
        Serial.println("Magnet on");
        break;
      case 0:
      default:
        digitalWrite(crtl_pin, LOW);
        Serial.println("Magnet off");
        break;
    }
  }
}
