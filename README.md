# System Overview: GLOF Monitoring and Control Using IoT and Remote Data Analysis
#### This document outlines a professional approach to designing a monitoring system for Glacial Lake Outburst Floods (GLOF). The system integrates IoT devices, sensors, and a centralized server for real-time data collection, processing, and visualization.
---

## System Architecture
Sensor Integration
* Use specialized sensors (e.g., water level, temperature, humidity, pressure) to monitor glacial lake parameters.
* Connect these sensors to a Wi-Fi-enabled microcontroller like the NodeMCU (ESP8266) or ESP32 for efficient data transmission.
## Local Data Transfer
* Sensors send collected data to the NodeMCU, which acts as a local hub.
* The NodeMCU communicates via Wi-Fi to transmit sensor data to the central server in real-time.
## Centralized Data Processing
The server, a remote PC or laptop, is configured to:
* Run Ubuntu, a stable and widely-used operating system for development and deployment.
* Host a database or file system to store incoming data.
Process and visualize data using ros tool plotjuggler.
Visualization and Alerts


## Threshold-Based Alerts and Data Monitoring System with Updated Sensor Setup
This guide outlines a comprehensive approach to configure NodeMCU for multi-sensor data collection, set up a server for processing the data, and implement threshold-based alerts to notify stakeholders in case of anomalies. It also integrates the updated list of sensors for Glacial Lake Outburst Flood (GLOF) monitoring.

## Configure NodeMCU for Sensor Data Collection
Updated Sensors and Their Purposes<br>
### Sensors and Their Purposes

| **Sensor**             | **Purpose**                     | **Connection Pins**         |
|-------------------------|----------------------------------|-----------------------------|
| **HC-SR04 Ultrasonic** | Water level monitoring          | GPIO Trigger, Echo          |
| **DHT22**              | Air temperature & humidity      | GPIO Data                   |
| **DS18B20**            | Water temperature               | GPIO Data with resistor     |
| **YF-S201 Flow Sensor**| Water flow rate                 | GPIO Input                  |
| **TSD-10 Turbidity**   | Water clarity                   | Analog Input (ADC)          |
| **BMP280**             | Atmospheric pressure and altitude | I2C (SCL, SDA)             |

#### Install Required Libraries in Arduino IDE
Install the following libraries:

BNO055: Adafruit_Sensor and Adafruit_BNO055.<br>
DHT22: DHT library.<br>
DS18B20: OneWire and DallasTemperature.<br>
BMP280: Adafruit_BMP280.<br>
ArduinoJson: For JSON serialization.<br>
WebSockets2_Generic: For WebSocket communication.<br>
```bash
#include <Wire.h>
#include <WebSockets2_Generic.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP280.h>

// WiFi and Server Configuration
const char* ssid = "Your_SSID";
const char* password = "Your_Password";
const char* websockets_server_host = "192.168.x.xxx";
const uint16_t websockets_server_port = 8080;

// Pin Definitions
#define DHTPIN 5
#define DHTTYPE DHT22
#define ONE_WIRE_BUS 4
#define TURBIDITY_PIN A0
#define ULTRASONIC_TRIGGER_PIN 12
#define ULTRASONIC_ECHO_PIN 13

// Threshold Values
#define THRESHOLD_TEMP 30.0
#define THRESHOLD_HUMIDITY 70.0
#define THRESHOLD_WATER_TEMP 15.0
#define THRESHOLD_FLOW_RATE 50.0
#define THRESHOLD_TURBIDITY 300

// Sensor Initialization
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature waterTempSensor(&oneWire);
Adafruit_BMP280 bmp; // For pressure and altitude
StaticJsonDocument<1024> doc;

WebsocketsClient client;

void setup() {
  Serial.begin(38400);

  // Initialize Sensors
  dht.begin();
  waterTempSensor.begin();
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not detected!");
    while (1);
  }

  // Ultrasonic Sensor Setup
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  // Connect to WebSocket Server
  client.connect(websockets_server_host, websockets_server_port, "/sensor");
}

void loop() {
  // Read Sensors
  float airTemp = dht.readTemperature();
  float humidity = dht.readHumidity();
  float waterTemp = waterTempSensor.getTempCByIndex(0);
  float pressure = bmp.readPressure() / 100.0F;
  float altitude = bmp.readAltitude(1013.25);

  // Ultrasonic Sensor - Water Level
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  float duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  float waterLevel = (duration / 2) * 0.0343;

  // Turbidity Sensor
  int turbidity = analogRead(TURBIDITY_PIN);

  // JSON Data Preparation
  doc["airTemp"] = airTemp;
  doc["humidity"] = humidity;
  doc["waterTemp"] = waterTemp;
  doc["pressure"] = pressure;
  doc["altitude"] = altitude;
  doc["waterLevel"] = waterLevel;
  doc["turbidity"] = turbidity;

  // Check for Thresholds
  if (airTemp > THRESHOLD_TEMP || humidity > THRESHOLD_HUMIDITY) {
    doc["alert"] = "Weather anomaly detected!";
  }
  if (waterTemp > THRESHOLD_WATER_TEMP) {
    doc["alert"] = "Water temperature anomaly detected!";
  }
  if (turbidity > THRESHOLD_TURBIDITY) {
    doc["alert"] = "Water turbidity anomaly detected!";
  }

  // Send Data via WebSocket
  String jsonString;
  serializeJson(doc, jsonString);
  client.send(jsonString);

  delay(1000); // Adjust data transmission frequency as needed
}

```

### Set Up the Server
Install Ubuntu on the Server

Download the latest Ubuntu ISO from Ubuntu Official Website.
Install Ubuntu on the remote PC or laptop and ensure it has internet connectivity.

## Prerequisites

Before starting, ensure the following are installed:

- **ROS Noetic**: Use the command <br>
```bash
wget-c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh <br>to set up ROS Noetic.<br>
```
- **Catkin tools**: Installed with the ROS installation.

---

## Installation Steps

### 1. Install Required Packages

Update your system and install essential tools:
```bash
sudo apt update
sudo apt install -y build-essential python3-rosdep python3-catkin-tools python3-vcstool git
```
### 2. Initialize rosdep
If you haven’t already initialized rosdep, do so:
```bash
sudo rosdep init
rosdep update
```
### 3. Create a ROS Workspace
Create the workspace directory and navigate to it:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
```
### 4. Build the Workspace
Run catkin_make to initialize and build the workspace:
```bash
catkin_make
```
### 5. Source the Workspace
After a successful build, source the workspace to make it available in your shell:
```bash
source devel/setup.bash
```
### 6. Add Workspace to Bashrc
To source the workspace automatically on shell startup:
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## Adding ROS Packages
### 1. Navigate to the src directory:
```bash
cd ~/catkin_ws/src
```
### 2. Clone the package repositories or add your custom packages:
```bash
git clone <repository_url>
```
### 3. Resolve dependencies:
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
### 4. Rebuild the workspace:
```bash
catkin_make
```
##### Troubleshooting<br>
If catkin_make fails:

Ensure all required dependencies are installed using rosdep.<br>
For Python dependencies, use pip with virtualenv to isolate them.

Verification<br>
To verify that your workspace is set up correctly, run:
```bash
roscd
```

This should navigate to your workspace's src directory.
### 5. Install Required Python Libraries: 
Use pip to install dependencies:
```bash
pip3 install pandas websockets
```
### 6. Install PlotJuggler: 
Install PlotJuggler for ROS using:
```bash
sudo snap install plotjuggler-ros
```
Replace your_package with the name of the ROS package where this script is located.

### 7. Use PlotJuggler: Launch PlotJuggler to visualize the data:
```bash
plotjuggler
```
## Steps to Create and Add the Code File:
Create a ROS Package: If you don’t have a ROS package yet, create one using the following steps:

### 1. Navigate to your ROS workspace:
```bash
cd ~/catkin_ws/src
```
### 2. Create a new package (replace your_package_name with your desired package name):
```bash
catkin_create_pkg your_package_name rospy std_msgs
```
### 3. Change to the new package directory:
```bash
cd ~/catkin_ws/src/your_package_name
```
### 4. Create the Python Script:
Inside the package, create a new Python script (e.g., websocket_to_ros.py) in the scripts directory:
```bash
mkdir scripts
cd scripts
touch websocket_to_ros.py
chmod +x websocket_to_ros.py  # Make it executable
```
### 5. Add the Code to the Python Script: Open the newly created websocket_to_ros.py file with any text editor (e.g., nano or vim):
```bash
nano websocket_to_ros.py
```
Copy and paste the following code inside the file:
```bash
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import websockets
import asyncio
import pandas as pd
import json
from datetime import datetime

class WebSocketToRosNode:
    def __init__(self):
        rospy.init_node('plotjuggler', anonymous=True)

        # Define the ROS 1 publisher
        self.publishers = {}

        # Initialize Pandas DataFrame and CSV file for logging
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = 'log_plot_data_{}.csv'.format(self.timestamp)
        self.df = pd.DataFrame(columns=['timestamp', 'key', 'value'])

        # Start the WebSocket server
        self.loop = asyncio.get_event_loop()
        start_server = websockets.serve(self.websocket_handler, "localhost", 5000)
        self.loop.run_until_complete(start_server)
        self.loop.run_forever()

    async def websocket_handler(self, websocket, path):
        async for message in websocket:
            rospy.loginfo("Received message: {}".format(message))
            self.process_data(message)

    def process_data(self, data):
        try:
            # Parse JSON data
            json_data = json.loads(data)
            rospy.loginfo("Parsed JSON data: {}".format(json_data))

            # Extract key and value
            for key, value in json_data.items():
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                # Log data to DataFrame
                new_row = pd.DataFrame({'timestamp': [timestamp], 'key': [key], 'value': [value]})
                self.df = pd.concat([self.df, new_row], ignore_index=True)

                # Publish data to ROS 1 topic
                if key not in self.publishers:
                    self.publishers[key] = rospy.Publisher('plot_data/{}'.format(key), String, queue_size=10)

                msg = String()
                msg.data = '{}: {}'.format(key, value)
                self.publishers[key].publish(msg)
                rospy.loginfo("Publishing to plot_data/{}: {}".format(key, value))

                # Save DataFrame to CSV file
                if not self.df.empty:
                    self.df.to_csv(self.csv_filename, index=False)
                    rospy.loginfo("DataFrame saved to {}".format(self.csv_filename))
                else:
                    rospy.logwarn("DataFrame is empty; no data to save.")

        except json.JSONDecodeError:
            rospy.logerr('Failed to decode JSON: {}'.format(data))
        except Exception as e:
            rospy.logerr('Error processing data: {}'.format(str(e)))

if __name__ == '__main__':
    try:
        WebSocketToRosNode()
    except rospy.ROSInterruptException:
        pass
```
### 6. Save and Close the File:

In nano, press CTRL + X, then Y to confirm changes, and Enter to save the file.
### 7. Make the Script Executable: If you haven't already, ensure that the Python script is executable:
```bash
chmod +x scripts/websocket_to_ros.py
```
### 8. Update CMakeLists.txt and package.xml: 
Ensure the CMakeLists.txt and package.xml files are updated to include the Python script.
* Open CMakeLists.txt and ensure the following lines are included to enable Python scripts:
```bash
catkin_install_python(PROGRAMS scripts/websocket_to_ros.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```
* Open package.xml and ensure the following dependencies are listed:
```bash
<exec_depend>rospy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
### 9. Build Your Workspace:
Go back to the root of your workspace and build:
```bash
cd ~/catkin_ws
catkin_make
```
### 10. Source the workspace:
```bash
source devel/setup.bash
```
### 11. Run the Node: 
After building, run the Python script using rosrun:
```bash
rosrun your_package_name websocket_to_ros.py
```
Replace your_package_name with the actual package name you created earlier.

# YOLOv5 Quickstart Guide For the Lake Detection Model

## Documentation
For full documentation on training, testing, and deployment, please refer to the [YOLOv5 Documentation](https://github.com/ultralytics/yolov5/wiki).

## Installation
Follow these steps to set up YOLOv5 in your environment:

### Requirements
- Python >= 3.8.0
- PyTorch >= 1.8

### Steps
1. Clone the YOLOv5 repository:
   ```bash
   git clone https://github.com/ultralytics/yolov5
2. Navigate to the cloned directory:
   ```bash
   cd yolov5
3. Install the required dependencies:
   Install all the necessary packages listed in the `requirements.txt` file:
   ```bash
   pip install -r requirements.txt
4. Verify the installation:  
   Use the following command to check if YOLOv5 is correctly installed. This command runs object detection on an example image using a pre-trained YOLOv5s model:  
   ```bash
   python detect.py --source 0 --weights lake.pt --conf 0.25


## 🚀 Project Completion

Congratulations! You have successfully set up a ROS package, integrated WebSocket communication, and implemented real-time sensor data logging and visualization. This project demonstrates your ability to work with ROS, sensor data integration, and real-time data visualization, showcasing your technical skills and problem-solving capabilities.

# **GLOF Detection System** 🚀

## **Project Overview**

This project involves the integration of multiple environmental sensors with a WebSocket-based communication system for real-time data streaming and logging. The goal is to gather sensor data, visualize it using PlotJuggler, and process it for GLOF (Glacial Lake Outburst Flood) detection.

## **Key Features**
- Integration of various sensors (BNO055, DHT22, BMP280, etc.) for monitoring critical parameters.
- Real-time data streaming using WebSockets.
- Data logging to CSV for historical data analysis.
- Real-time data visualization using PlotJuggler.

---

## 🚀 **Project Completion**
### **Key Achievements**:
- **Implemented WebSocket-based communication** for real-time data streaming.
- **Developed a data logging system** using **CSV** for sensor data.
- **Integrated multiple sensors** for various environmental parameters.
- **Utilized PlotJuggler ROS** for **real-time data visualization**.
- Used Micro-controller(Jetson-Nano)

This project highlights your **expertise in ROS**, **IoT sensor integration**, and **data handling**, making you well-equipped for further challenges in **robotics** and **automation**.

