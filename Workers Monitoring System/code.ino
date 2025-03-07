#include <Wire.h>               
#include <TinyGPS++.h>         
#include <MPU6050.h>           
#include <WiFi.h>              
#include <ThingSpeak.h>        
#include <PulseSensorPlayground.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// WiFi Credentials
const char* ssid = "YOUR_CONNECTION_ID";      
const char* password = "YOUR_CONNECTION_"; 

// ThingSpeak Credentials
unsigned long channelID = YOUR_CHANNEL_ID;  
const char* writeAPIKey = "CHANNEL_WRITE_API_KEY"; 

WiFiClient client;  
WiFiClientSecure secure_client;

// GPS Module (NEO-6M) pins
#define GPS_RX 32
#define GPS_TX 33 
TinyGPSPlus gps;   

// MPU6050 (Accelerometer)
MPU6050 mpu;       

// Pulse Sensor
const int pulsePin = 34;  
PulseSensorPlayground pulseSensor;

// Twilio Credentials
const char* twilio_account_sid = "TWILIO_SID";
const char* twilio_auth_token = "AUTH_TOKEN";
const char* from_number = "TWILIO_PHONE";  // Twilio Sandbox Number
const char* to_number = "YOUR_NUMBER";   // Supervisor's WhatsApp Number

// Geofencing parameters
const float safeLatitude = 12.881626;  // Set to your safe zone latitude
const float safeLongitude = 77.444617; // Set to your safe zone longitude
const float geoFenceRadius = 25;  // Radius in meters

// Inactivity Timer
unsigned long lastActiveTime = 0;
const unsigned long inactivityThreshold = 60000;  // 1 minute

// Function to calculate distance between two GPS coordinates
float haversineDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000; // Earth radius in meters
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) * 
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// Function to send WhatsApp alert
void sendWhatsAppMessage(String message) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    secure_client.setInsecure();  
    http.begin(secure_client, "https://api.twilio.com/2010-04-01/Accounts/" + String(twilio_account_sid) + "/Messages.json");
    http.setAuthorization(twilio_account_sid, twilio_auth_token);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "To=" + String(to_number) + "&From=" + String(from_number) + "&Body=" + urlencode(message);

    int httpResponseCode = http.POST(body);
    if (httpResponseCode == 201) {
      Serial.println("✅ SMS Alert Sent Successfully");
    } else {
      Serial.print("❌ SMS Failed, Error Code: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  }
}

String urlencode(String str) {
  String encodedString = "";
  char c;
  char code[3];
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (isalnum(c)) {
      encodedString += c;
    } else {
      sprintf(code, "%%%02X", c);
      encodedString += code;
    }
  }
  return encodedString;
}


void setup() {
  Serial.begin(115200);  
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); 

  Wire.begin(21, 22);     
  mpu.initialize();       
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed ❌");
  } else {
    Serial.println("MPU6050 initialized ✅");
  }

  pulseSensor.analogInput(pulsePin);
  pulseSensor.setThreshold(550);
  if (pulseSensor.begin()) {
    Serial.println("Pulse Sensor initialized ✅");
  } else {
    Serial.println("Pulse Sensor initialization failed ❌");
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi ✅");

  ThingSpeak.begin(client);  
}

void loop() {
  // Read GPS Data
  while (Serial2.available() > 0) { 
    gps.encode(Serial2.read());    
  }

  float lat = 0.0, lon = 0.0;
  bool gpsAvailable = false;
  if (gps.location.isUpdated()) {
    lat = gps.location.lat();   
    lon = gps.location.lng();   
    gpsAvailable = true;
  }

  // Read MPU6050 Acceleration
  int16_t ax, ay, az; 
  mpu.getAcceleration(&ax, &ay, &az); 
  float ax_g = ax / 16384.0;  
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float acceleration = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g); 

  int rawValue = analogRead(pulsePin);
  Serial.print("Raw Pulse Sensor Value: ");
  Serial.println(rawValue);
  delay(100);


  // Read Pulse Rate
  int sumBPM = 0;
  int numReadings = 5;
  for (int i = 0; i < numReadings; i++) {
    sumBPM += pulseSensor.getBeatsPerMinute();
    delay(20);
  }
  int pulseRate = sumBPM / numReadings;
  
  // Send Data to ThingSpeak
  ThingSpeak.setField(1, lat);  
  ThingSpeak.setField(2, lon);  
  ThingSpeak.setField(3, acceleration);  
  ThingSpeak.setField(4, pulseRate);

  int httpCode = ThingSpeak.writeFields(channelID, writeAPIKey);
  if (httpCode == 200) {
    Serial.println("✅ Data sent to ThingSpeak!");
  } else {
    Serial.println("❌ Failed to send data to ThingSpeak.");
  
  }
  


  // Display Data on Serial Monitor
  if (gpsAvailable) {
    Serial.print("📍 Latitude: "); Serial.println(lat);
    Serial.print("📍 Longitude: "); Serial.println(lon);
  } else {
    Serial.println("⚠️ GPS Data Not Available (Indoors or Poor Signal)");
  }
  Serial.print("⚡ Acceleration (g): "); Serial.println(acceleration);
  Serial.print("❤️ Pulse Rate (BPM): "); Serial.println(pulseRate);

  // Send Alerts
  if (pulseRate > 250) {
    sendWhatsAppMessage("🚨 ALERT: High Pulse Rate Detected! BPM: " + String(pulseRate));
    delay(10000);
  }
  if (pulseRate < 45) {
    sendWhatsAppMessage("⚠️ ALERT: Low Pulse Rate Detected! BPM: " + String(pulseRate));
    delay(10000);
  }

  // Geofencing Alert
  if (gpsAvailable) {
    float distanceFromSafeZone = haversineDistance(lat, lon, safeLatitude, safeLongitude);
    if (distanceFromSafeZone > geoFenceRadius) {
      sendWhatsAppMessage("🚧 ALERT: Worker left the geofenced area! Distance: " + String(distanceFromSafeZone) + " meters.");
      delay(10000);
    }  
  }

  // Inactivity Alert
  if (acceleration < 1.2) {
    if (millis() - lastActiveTime > inactivityThreshold) {
      sendWhatsAppMessage("⚠️ ALERT: Worker inactive for more than 1 minute.");
      lastActiveTime = millis();  
    }
  } else {
    lastActiveTime = millis();  
  }