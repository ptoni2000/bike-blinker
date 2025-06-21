// Define the pins for buttons and outputs
const int BUTTON1_PIN = A2; // Using Analog Pin A2 for Button 1
const int BUTTON2_PIN = A1; // Using Analog Pin A1 for Button 2
const int OUTPUT1_PIN = 0;
const int OUTPUT2_PIN = 1;

// Debounce parameters
const unsigned long DEBOUNCE_DELAY_MS = 50; // milliseconds

// Analog read threshold for buttons
const int ANALOG_THRESHOLD = 256; // Values above this are considered HIGH (pressed)

const unsigned long BLINK_INTERVAL_MS = 500; // Blink interval (half-period: 500ms ON, 500ms OFF)

// Define system states
enum SystemState {
  IDLE,
  BLINKING_OUTPUT1,
  BLINKING_OUTPUT2,
  HAZARD
};
SystemState currentState = IDLE; // Initial state

// Struct to hold all state variables for a button
struct ButtonState {
  int pin;
  int debouncedState;
  int lastReading;
  unsigned long lastDebounceTime;
};

// Struct to hold all state variables for an output
struct OutputState {
  int pin;
  int ledState;
  unsigned long lastBlinkTime;
};

// Create instances of the structs for our buttons and outputs
ButtonState button1 = {BUTTON1_PIN, LOW, LOW, 0};
ButtonState button2 = {BUTTON2_PIN, LOW, LOW, 0};

OutputState output1 = {OUTPUT1_PIN, LOW, 0};
OutputState output2 = {OUTPUT2_PIN, LOW, 0};

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
  pinMode(output1.pin, OUTPUT);
  pinMode(output2.pin, OUTPUT);

  // Ensure outputs are initially OFF and LED states reflect this
  digitalWrite(output1.pin, LOW);
  output1.ledState = LOW;
  digitalWrite(output2.pin, LOW);
  output2.ledState = LOW;

  // Initialize button debouncing states based on initial readings
  button1.lastReading = readAnalogButton(button1.pin);
  button1.debouncedState = button1.lastReading;
  button2.lastReading = readAnalogButton(button2.pin);
  button2.debouncedState = button2.lastReading;

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
// btn: Reference to the ButtonState struct for the button being checked
// Returns: true if a debounced press event (LOW to HIGH transition) occurred, false otherwise
bool checkButtonPressed(ButtonState& btn) {
  unsigned long currentTime = millis();
  bool pressEvent = false;
  int reading = readAnalogButton(btn.pin);

  if (reading != btn.lastReading) {
    btn.lastDebounceTime = currentTime;
  }

  if ((currentTime - btn.lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (reading != btn.debouncedState) {
      btn.debouncedState = reading;
      if (btn.debouncedState == HIGH) {
        pressEvent = true; // Detected a press (rising edge)
      }
    }
  }
  btn.lastReading = reading;
  return pressEvent;
}

void loop() {
  unsigned long currentTime = millis();

  // --- Read and Debounce Buttons ---
  bool button1PressedEvent = checkButtonPressed(button1);
  bool button2PressedEvent = checkButtonPressed(button2);

  // --- State Transition Logic ---

  // Check for a "hazard" event first (one button pressed while the other is already held down).
  // This is a more reliable way to detect a "both buttons" press than checking for simultaneous press events.
  bool hazardEvent = (button1PressedEvent && button2.debouncedState == HIGH) || (button2PressedEvent && button1.debouncedState == HIGH);

  if (hazardEvent) {
    if (currentState != HAZARD) {
      currentState = HAZARD;
      // Start blinking with LEDs ON for immediate feedback
      output1.ledState = HIGH;
      output2.ledState = HIGH;
      digitalWrite(output1.pin, output1.ledState);
      digitalWrite(output2.pin, output2.ledState);
      // Use one timer for both hazard lights
      output1.lastBlinkTime = currentTime;
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
        output1.ledState = HIGH; // Start blinking with LED ON
        digitalWrite(output1.pin, output1.ledState); // Immediate visual feedback
        output1.lastBlinkTime = currentTime; // Reset blink timer
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
        output2.ledState = HIGH; // Start blinking with LED ON
        digitalWrite(output2.pin, output2.ledState); // Immediate visual feedback
        output2.lastBlinkTime = currentTime; // Reset blink timer
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
      if (output1.ledState != LOW) {
        digitalWrite(output1.pin, LOW);
        output1.ledState = LOW;
        // Serial.println("IDLE: Output 1 OFF");
      }
      if (output2.ledState != LOW) {
        digitalWrite(output2.pin, LOW);
        output2.ledState = LOW;
        // Serial.println("IDLE: Output 2 OFF");
      }
      break;

    case BLINKING_OUTPUT1:
      // Ensure Output 2 is OFF
      if (output2.ledState != LOW) {
        digitalWrite(output2.pin, LOW);
        output2.ledState = LOW;
        // Serial.println("BLINKING_OUTPUT1: Output 2 OFF");
      }
      // Blink Output 1
      if (currentTime - output1.lastBlinkTime >= BLINK_INTERVAL_MS) {
        output1.lastBlinkTime = currentTime;
        output1.ledState = !output1.ledState; // Toggle
        digitalWrite(output1.pin, output1.ledState);
        // Serial.print("BLINKING_OUTPUT1: Output 1 Toggled to "); Serial.println(output1.ledState);
      }
      break;

    case BLINKING_OUTPUT2:
      // Ensure Output 1 is OFF
      if (output1.ledState != LOW) {
        digitalWrite(output1.pin, LOW);
        output1.ledState = LOW;
        // Serial.println("BLINKING_OUTPUT2: Output 1 OFF");
      }
      // Blink Output 2
      if (currentTime - output2.lastBlinkTime >= BLINK_INTERVAL_MS) {
        output2.lastBlinkTime = currentTime;
        output2.ledState = !output2.ledState; // Toggle
        digitalWrite(output2.pin, output2.ledState);
        // Serial.print("BLINKING_OUTPUT2: Output 2 Toggled to "); Serial.println(output2.ledState);
      }
      break;

    case HAZARD:
      // Blink both outputs simultaneously using one timer
      if (currentTime - output1.lastBlinkTime >= BLINK_INTERVAL_MS) {
        output1.lastBlinkTime = currentTime;
        // Toggle both LED states
        output1.ledState = !output1.ledState;
        output2.ledState = output1.ledState; // Keep them in sync
        digitalWrite(output1.pin, output1.ledState);
        digitalWrite(output2.pin, output2.ledState);
        // Serial.print("HAZARD: Outputs Toggled to "); Serial.println(output1.ledState);
      }
      break;
  }
}
