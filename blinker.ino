// Define the pins for buttons and outputs
const int BUTTON1_PIN = 2;
const int BUTTON2_PIN = 3;
const int OUTPUT1_PIN = 4;
const int OUTPUT2_PIN = 5;

// Debounce parameters
const unsigned long DEBOUNCE_DELAY_MS = 50; // milliseconds

// Button 1 state variables
int button1State = LOW;           // Current debounced state of button 1 (HIGH when pressed)
int lastButton1Reading = LOW;     // Last raw reading of button 1
unsigned long lastDebounceTimeButton1 = 0; // Last time button 1's reading changed

// Button 2 state variables
int button2State = LOW;           // Current debounced state of button 2 (HIGH when pressed)
int lastButton2Reading = LOW;     // Last raw reading of button 2
unsigned long lastDebounceTimeButton2 = 0; // Last time button 2's reading changed

// Output blinking control
bool output1ShouldBlink = false;
bool output2ShouldBlink = false;

// LED state for blinking logic
int output1LedState = LOW; // Current physical state of Output 1 (HIGH/LOW)
int output2LedState = LOW; // Current physical state of Output 2 (HIGH/LOW)
unsigned long lastBlinkTimeOutput1 = 0; // Last time Output 1 toggled
unsigned long lastBlinkTimeOutput2 = 0; // Last time Output 2 toggled
const unsigned long BLINK_INTERVAL_MS = 500; // Blink interval (half-period: 500ms ON, 500ms OFF)

void setup() {
  // Optional: Initialize Serial communication for debugging
  // Serial.begin(9600);
  // Serial.println("System Initialized");

  // Initialize button pins as inputs.
  // IMPORTANT: This code assumes buttons are wired with external pull-DOWN resistors.
  // When a button is pressed, it connects VCC to the input pin (making it HIGH).
  // When not pressed, the pull-down resistor pulls the pin LOW.
  // Example wiring: VCC --- Button --- PinX
  //                               |
  //                             10k Ohm Resistor
  //                               |
  //                              GND
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);

  // Initialize output pins as outputs
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);

  // Ensure outputs are initially OFF
  digitalWrite(OUTPUT1_PIN, LOW);
  digitalWrite(OUTPUT2_PIN, LOW);
  output1LedState = LOW;
  output2LedState = LOW;

  // Initialize button states based on initial readings
  lastButton1Reading = digitalRead(BUTTON1_PIN);
  button1State = lastButton1Reading;
  lastButton2Reading = digitalRead(BUTTON2_PIN);
  button2State = lastButton2Reading;
}

void loop() {
  unsigned long currentTime = millis();

  // --- Handle Button 1 Input and Debouncing ---
  int currentButton1Reading = digitalRead(BUTTON1_PIN);

  // If the reading has changed, reset the debounce timer
  if (currentButton1Reading != lastButton1Reading) {
    lastDebounceTimeButton1 = currentTime;
  }

  // If the reading has been stable for longer than the debounce delay
  if ((currentTime - lastDebounceTimeButton1) > DEBOUNCE_DELAY_MS) {
    // If the button's debounced state has changed
    if (currentButton1Reading != button1State) {
      button1State = currentButton1Reading; // Update the debounced state

      // Check if the button was pressed (transitioned to HIGH)
      if (button1State == HIGH) {
        // Serial.println("Button 1 Pressed");
        if (output1ShouldBlink || output2ShouldBlink) {
          // If any output is blinking, turn all blinking off and set outputs to LOW
          // Serial.println("  Action: Turning all blinking off.");
          output1ShouldBlink = false;
          output2ShouldBlink = false;
          digitalWrite(OUTPUT1_PIN, LOW);
          digitalWrite(OUTPUT2_PIN, LOW);
          output1LedState = LOW;
          output2LedState = LOW;
        } else {
          // No output is blinking, make Output 1 blink
          // Serial.println("  Action: Setting Output 1 to blink.");
          output1ShouldBlink = true;
          output2ShouldBlink = false; // Ensure Output 2 is not set to blink
          
          // Explicitly turn Output 2 OFF if it was somehow ON (but not blinking)
          if (output2LedState == HIGH) {
             digitalWrite(OUTPUT2_PIN, LOW);
             output2LedState = LOW;
          }
        }
      }
    }
  }
  lastButton1Reading = currentButton1Reading; // Save the current reading for the next loop iteration

  // --- Handle Button 2 Input and Debouncing ---
  int currentButton2Reading = digitalRead(BUTTON2_PIN);

  if (currentButton2Reading != lastButton2Reading) {
    lastDebounceTimeButton2 = currentTime;
  }

  if ((currentTime - lastDebounceTimeButton2) > DEBOUNCE_DELAY_MS) {
    if (currentButton2Reading != button2State) {
      button2State = currentButton2Reading;

      if (button2State == HIGH) { // Button 2 was pressed
        // Serial.println("Button 2 Pressed");
        if (output1ShouldBlink || output2ShouldBlink) {
          // If any output is blinking, turn all blinking off and set outputs to LOW
          // Serial.println("  Action: Turning all blinking off.");
          output1ShouldBlink = false;
          output2ShouldBlink = false;
          digitalWrite(OUTPUT1_PIN, LOW);
          digitalWrite(OUTPUT2_PIN, LOW);
          output1LedState = LOW;
          output2LedState = LOW;
        } else {
          // No output is blinking, make Output 2 blink
          // Serial.println("  Action: Setting Output 2 to blink.");
          output2ShouldBlink = true;
          output1ShouldBlink = false; // Ensure Output 1 is not set to blink

          // Explicitly turn Output 1 OFF if it was somehow ON (but not blinking)
          if (output1LedState == HIGH) {
             digitalWrite(OUTPUT1_PIN, LOW);
             output1LedState = LOW;
          }
        }
      }
    }
  }
  lastButton2Reading = currentButton2Reading;

  // --- Update Blinking Outputs ---

  // Handle Output 1 blinking
  if (output1ShouldBlink) {
    if (currentTime - lastBlinkTimeOutput1 >= BLINK_INTERVAL_MS) {
      lastBlinkTimeOutput1 = currentTime;
      output1LedState = !output1LedState; // Toggle LED state
      digitalWrite(OUTPUT1_PIN, output1LedState);
      // Serial.print("Output 1 toggled to: "); Serial.println(output1LedState == HIGH ? "ON" : "OFF");
    }
  } else {
    // If Output 1 is not supposed to blink, ensure its physical state matches output1LedState (which should be LOW).
    // This is primarily handled when output1ShouldBlink is set to false,
    // where digitalWrite(OUTPUT1_PIN, LOW) and output1LedState = LOW are called.
    // This 'else' block can be a safeguard but is often redundant if the state management is correct.
    // For simplicity and clarity, relying on the explicit OFF commands when blinking stops.
  }

  // Handle Output 2 blinking
  if (output2ShouldBlink) {
    if (currentTime - lastBlinkTimeOutput2 >= BLINK_INTERVAL_MS) {
      lastBlinkTimeOutput2 = currentTime;
      output2LedState = !output2LedState; // Toggle LED state
      digitalWrite(OUTPUT2_PIN, output2LedState);
      // Serial.print("Output 2 toggled to: "); Serial.println(output2LedState == HIGH ? "ON" : "OFF");
    }
  } else {
    // Similar to Output 1, ensure it's off if not blinking.
  }
}
