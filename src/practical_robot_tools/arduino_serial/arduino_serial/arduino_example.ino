#define LED_PIN 13  // Using the built-in LED on most Arduino boards

void setup() {
    Serial.begin(9600);  // Start serial communication
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    if (Serial.available() > 0) {  // Check if data is available to read
        char receivedChar = Serial.read();  // Read the incoming data

        // Ensure the received character is valid
        if (receivedChar == '0') {
            digitalWrite(LED_PIN, LOW);  // Turn off the LED
            Serial.println("LED OFF");
        } 
        else if (receivedChar == '1') {
            digitalWrite(LED_PIN, HIGH);  // Turn on the LED
            Serial.println("LED ON");
        } 
        else if (receivedChar == '2') {
            Serial.println("Received 2: No LED action.");
        } 
        else {
            Serial.println("Invalid input received.");
        }
    }
}
