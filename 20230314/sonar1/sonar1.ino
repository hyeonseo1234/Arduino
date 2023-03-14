#define TRIG1 23   // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 24  // 초음파 센서 1번 Trig 핀 번호


void setup() {
  // put your setup code here, to run once:
  pinMode (TRIG1, OUTPUT);
  pinMode (ECHO1, INPUT);

  Serial.begin(115200); // 통신속도를 115200으로 정의함
  
}

void loop() {
  // put your main code here, to run repeatedly:
  long duration, distance;
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  duration = pulseIn(ECHO1, HIGH);
  Serial.print("Duration: ");
  Serial.println(duration);
  delay(500);

}
