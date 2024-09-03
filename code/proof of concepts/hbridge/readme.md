# H-Bridge proof of concept

// Motor Rechts
const int IN1 = 2;
const int IN2 = 3;

// Motor Links
const int IN3 = 4;
const int IN4 = 5;

// Variabelen
bool StatusKnop;

void setup() {
  Serial.begin(9600);

  // Digitaal

  pinMode(IN1, OUTPUT); //GroenBatterij
  pinMode(IN2, OUTPUT); //OranjeBatterij
  pinMode(IN3, OUTPUT); //GroenBatterij
  pinMode(IN4, OUTPUT); //OranjeBatterij
}

void loop() {


    // Voorwaarts
    
  for (int i = 0; i <= 255; i++)
    {
      analogWrite(IN1, i);
      analogWrite(IN2, 0);
      analogWrite(IN3, 0);
      analogWrite(IN4, i);
      delay(10);
    }
    for (int i = 255; i >= 0; i--)
    {
      analogWrite(IN1, i);
      analogWrite(IN2, 0);
      analogWrite(IN3, 0);
      analogWrite(IN4, i);
      delay(10);
    }

  delay(2000);

    // Achterwaarts
    
  for (int i = 0; i <= 255; i++)
    {
      analogWrite(IN1, 0);
      analogWrite(IN2, i);
      analogWrite(IN3, i);
      analogWrite(IN4, 0);
      delay(10);
    }
    for (int i = 255; i >= 0; i--)
    {
      analogWrite(IN1, 0);
      analogWrite(IN2, i);
      analogWrite(IN3, i);
      analogWrite(IN4, 0);
      delay(10);
    }

  delay(2000);

    // Links
    
  for (int i = 0; i <= 255; i++)
    {
      analogWrite(IN1, i);
      analogWrite(IN2, 0);
      analogWrite(IN3, i);
      analogWrite(IN4, 0);
      delay(10);
    }
    for (int i = 255; i >= 0; i--)
    {
      analogWrite(IN1, i);
      analogWrite(IN2, 0);
      analogWrite(IN3, i);
      analogWrite(IN4, 0);
      delay(10);
    }

  delay(2000);

    // Rechts
    
  for (int i = 0; i <= 255; i++)
    {
      analogWrite(IN1, 0);
      analogWrite(IN2, i);
      analogWrite(IN3, 0);
      analogWrite(IN4, i);
      delay(10);
    }
    for (int i = 255; i >= 0; i--)
    {
      analogWrite(IN1, 0);
      analogWrite(IN2, i);
      analogWrite(IN3, 0);
      analogWrite(IN4, i);
      delay(10);
    }

  
}
