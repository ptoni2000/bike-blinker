// Define the pins for buttons and outputs
const int BUTTON1_PIN = A2; // Using Analog Pin A2 for Button 1
const int BUTTON2_PIN = A1; // Using Analog Pin A1 for Button 2
const int OUTPUT1_PIN = 0;
const int OUTPUT2_PIN = 1;

// Debounce parameters
const unsigned long DEBOUNCE_DELAY_MS = 50; // milliseconds

// Analog read threshold
const int ANALOG_THRESHOLD = 256; // Values above this are considered HIGH (pressed)

// Button 1 state variables for debouncing
int debouncedButton1State = LOW;     // Current debounced state of button 1 (HIGH when pressed)
int lastButton1Reading = LOW;        // Last raw reading of button 1
unsigned long lastDebounceTimeButton1 = 0; // Last time button 1's reading changed

// Button 2 state variables for debouncing
int debouncedButton2State = LOW;     // Current debounced state of button 2 (HIGH when pressed)
int lastButton2Reading = LOW;        // Last raw reading of button 2
unsigned long lastDebounceTimeButton2 = 0; // Last time button 2's reading changed

// LED state for blinking logic
int output1LedState = LOW; // Current physical state of Output 1 (HIGH/LOW)
int output2LedState = LOW; // Current physical state of Output 2 (HIGH/LOW)
unsigned long lastBlinkTimeOutput1 = 0; // Last time Output 1 toggled
unsigned long lastBlinkTimeOutput2 = 0; // Last time Output 2 toggled
const unsigned long BLINK_INTERVAL_MS = 500; // Blink interval (half-period: 500ms ON, 500ms OFF)

// Define system states
enum SystemState {
  IDLE,
  BLINKING_OUTPUT1,
  BLINKING_OUTPUT2,
  HAZARD
};
SystemState currentState = IDLE; // Initial state

void setup() {
  // Optional: Initialize Serial communication for debugging
  // Serial.begin(9600);
  // Serial.println("System Initialized");

  // Initialize button pins as inputs.
  // IMPORTANT: This code assumes buttons are wired with external pull-DOWN resistors to an ANALOG pin.
  // When a button is pressed, it connects VCC to the input pin (making it HIGH).
  // When not pressed, the pull-down resistor pulls the pin LOW. (This is correct)
  // Example wiring: VCC --- Button --- 4k7---+ PinAX (e.g., A0)
  //                                          |
  //                                         1k Ohm Resistor (Pull-down)
  //                                          |
  //                                         GND
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);

  // Initialize output pins as outputs
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);

  // Ensure outputs are initially OFF and LED states reflect this
  digitalWrite(OUTPUT1_PIN, LOW);
  output1LedState = LOW;
  digitalWrite(OUTPUT2_PIN, LOW);
  output2LedState = LOW;

  // Initialize button debouncing states based on initial readings
  lastButton1Reading = readAnalogButton(BUTTON1_PIN);
  debouncedButton1State = lastButton1Reading;
  lastButton2Reading = readAnalogButton(BUTTON2_PIN);
  debouncedButton2State = lastButton2Reading;

  currentState = IDLE; // Set initial system state
  // Serial.println("Initial State: IDLE");
}

// Helper function to read an analog pin and convert to a digital state (HIGH/LOW)
// This abstracts the analog reading logic.
int readAnalogButton(int pin) {
  return (analogRead(pin) > ANALOG_THRESHOLD) ? HIGH : LOW;
}

// Helper function for debouncing a button and detecting a press event (rising edge).
// pin: The button pin number
// lastReading: Pointer to the variable storing the last raw reading of the button
// debouncedState: Pointer to the variable storing the current debounced state of the button
// lastDebounceTime: Pointer to the variable storing the last time the button reading changed
// Returns: true if a debounced press event (LOW to HIGH transition) occurred, false otherwise
bool checkButtonPressed(int pin, int &currentLastReading, int &currentDebouncedState, unsigned long &currentLastDebounceTime) {
  unsigned long currentTime = millis();
  bool pressEvent = false;
  int reading = readAnalogButton(pin);

  if (reading != currentLastReading) {
    currentLastDebounceTime = currentTime;
  }

  if ((currentTime - currentLastDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (reading != currentDebouncedState) {
      currentDebouncedState = reading;
      if (currentDebouncedState == HIGH) {
        pressEvent = true; // Detected a press (rising edge)
      }
    }
  }
  currentLastReading = reading;
  return pressEvent;
}

void loop() {
  unsigned long currentTime = millis();

  // --- Read and Debounce Buttons ---
  bool button1PressedEvent = checkButtonPressed(BUTTON1_PIN, lastButton1Reading, debouncedButton1State, lastDebounceTimeButton1);
  bool button2PressedEvent = checkButtonPressed(BUTTON2_PIN, lastButton2Reading, debouncedButton2State, lastDebounceTimeButton2);

  // --- State Transition Logic ---

  // Check for a "hazard" event first (one button pressed while the other is already held down).
  // This is a more reliable way to detect a "both buttons" press than checking for simultaneous press events.
  bool hazardEvent = (button1PressedEvent && debouncedButton2State == HIGH) || (button2PressedEvent && debouncedButton1State == HIGH);

  if (hazardEvent) {
    if (currentState != HAZARD) {
      currentState = HAZARD;
      // Start blinking with LEDs ON for immediate feedback
      output1LedState = HIGH;
      output2LedState = HIGH;
      digitalWrite(OUTPUT1_PIN, output1LedState);
      digitalWrite(OUTPUT2_PIN, output2LedState);
      // Use one timer for both hazard lights
      lastBlinkTimeOutput1 = currentTime;
      // Serial.println("  Transition to HAZARD");
    } else {
      // If already in HAZARD, a simultaneous press will turn it off.
      currentState = IDLE;
      // Serial.println("  Transition to IDLE from HAZARD");
    }
  } else if (button1PressedEvent) {
    // Serial.print("Button 1 Pressed Event. Current State: "); Serial.println(currentState);
    switch (currentState) {
      case IDLE:
        currentState = BLINKING_OUTPUT1;
        output1LedState = HIGH; // Start blinking with LED ON
        digitalWrite(OUTPUT1_PIN, output1LedState); // Immediate visual feedback
        lastBlinkTimeOutput1 = currentTime; // Reset blink timer
        // Serial.println("  Transition to BLINKING_OUTPUT1");
        break;
      case BLINKING_OUTPUT1:
      case BLINKING_OUTPUT2:
      case HAZARD: // A single press also cancels hazard mode
        currentState = IDLE;
        // Serial.println("  Transition to IDLE");
        break;
    }
  } else if (button2PressedEvent) {
    // Serial.print("Button 2 Pressed Event. Current State: "); Serial.println(currentState);
    switch (currentState) {
      case IDLE:
        currentState = BLINKING_OUTPUT2;
        output2LedState = HIGH; // Start blinking with LED ON
        digitalWrite(OUTPUT2_PIN, output2LedState); // Immediate visual feedback
        lastBlinkTimeOutput2 = currentTime; // Reset blink timer
        // Serial.println("  Transition to BLINKING_OUTPUT2");
        break;
      case BLINKING_OUTPUT1:
      case BLINKING_OUTPUT2:
      case HAZARD: // A single press also cancels hazard mode
        currentState = IDLE;
        // Serial.println("  Transition to IDLE");
        break;
    }
  }

  // --- Update Outputs Based on Current State ---
  switch (currentState) {
    case IDLE:
      // Ensure both outputs are OFF
      if (output1LedState != LOW) {
        digitalWrite(OUTPUT1_PIN, LOW);
        output1LedState = LOW;
        // Serial.println("IDLE: Output 1 OFF");
      }
      if (output2LedState != LOW) {
        digitalWrite(OUTPUT2_PIN, LOW);
        output2LedState = LOW;
        // Serial.println("IDLE: Output 2 OFF");
      }
      break;

    case BLINKING_OUTPUT1:
      // Ensure Output 2 is OFF
      if (output2LedState != LOW) {
        digitalWrite(OUTPUT2_PIN, LOW);
        output2LedState = LOW;
        // Serial.println("BLINKING_OUTPUT1: Output 2 OFF");
      }
      // Blink Output 1
      if (currentTime - lastBlinkTimeOutput1 >= BLINK_INTERVAL_MS) {
        lastBlinkTimeOutput1 = currentTime;
        output1LedState = !output1LedState; // Toggle
        digitalWrite(OUTPUT1_PIN, output1LedState);
        // Serial.print("BLINKING_OUTPUT1: Output 1 Toggled to "); Serial.println(output1LedState);
      }
      break;

    case BLINKING_OUTPUT2:
      // Ensure Output 1 is OFF
      if (output1LedState != LOW) {
        digitalWrite(OUTPUT1_PIN, LOW);
        output1LedState = LOW;
        // Serial.println("BLINKING_OUTPUT2: Output 1 OFF");
      }
      // Blink Output 2
      if (currentTime - lastBlinkTimeOutput2 >= BLINK_INTERVAL_MS) {
        lastBlinkTimeOutput2 = currentTime;
        output2LedState = !output2LedState; // Toggle
        digitalWrite(OUTPUT2_PIN, output2LedState);
        // Serial.print("BLINKING_OUTPUT2: Output 2 Toggled to "); Serial.println(output2LedState);
      }
      break;

    case HAZARD:
      // Blink both outputs simultaneously using one timer
      if (currentTime - lastBlinkTimeOutput1 >= BLINK_INTERVAL_MS) {
        lastBlinkTimeOutput1 = currentTime;
        // Toggle both LED states
        output1LedState = !output1LedState;
        output2LedState = output1LedState; // Keep them in sync
        digitalWrite(OUTPUT1_PIN, output1LedState);
        digitalWrite(OUTPUT2_PIN, output2LedState);
        // Serial.print("HAZARD: Outputs Toggled to "); Serial.println(output1LedState);
      }
      break;
  }
}
