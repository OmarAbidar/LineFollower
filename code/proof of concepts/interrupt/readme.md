# start/stop interrupt proof of concept
//minimale hard- en software die de correcte werking van een start/stop drukknop aantoont, gebruik makend van een hardware interrupt

const int buttonPin = 2;    // Pin where the button is connected
const int ledPin = 13;      // Pin where the LED is connected

int ledState = LOW;         // Initial state of the LED
int buttonState;            // Variable to hold the current button state
int lastButtonState = LOW;  // Variable to hold the last button state

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // Set button pin as input with an internal pull-up resistor
  pinMode(ledPin, OUTPUT);          // Set LED pin as output
}

void loop() {
  // Read the state of the button
  buttonState = digitalRead(buttonPin);

  // Check if the button has been pressed (state change)
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Toggle the LED state
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }

  // Update the last button state
  lastButtonState = buttonState;

  // Small delay for debounce
  delay(50);
}
