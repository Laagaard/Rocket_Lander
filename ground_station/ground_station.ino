
void  setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  Serial.println(1);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2500);
  Serial.write("");
  digitalWrite(LED_BUILTIN, LOW);
  delay(2500);
}