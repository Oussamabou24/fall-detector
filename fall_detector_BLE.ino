/*
This project is a fall detector. It is meant to be linked with an android app via BLE in order to send a notification to a predefined person if a fall is detected.
The used parameters to identify te fall are purly experimental and based on data observation of several falls.
The fall detection algorithm is based on three experementaly identified characteristics of a fall :
1- The free-fall phase, which preceeds the fall. The free-fall phase is identified first through a low acceleration threhold value which needs to stay below the limit for a certain amont of time
2- The impact phase, which is identified through :
  a-An acceleration peak which represents the impact moment
  b- A gyroscope value peak wich occurs in parallel with the acceleration peak

Once this scheme is identified, a fall is declared "detected". 

An output is then sent to an android app in order to trigger an emrgency message sending to a predefined person.

*/


#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>


BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service

// create switch characteristic and allow remote device to read and write
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead );

const int ledPin = LED_BUILTIN; // pin to use for the LED

const int numSamples = 119;

int samplesRead = numSamples;




void setup() {
  Serial.begin(9600);
  //while (!Serial);

  pinMode(ledPin, OUTPUT); // use the LED pin as an output

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }


  // set the local name peripheral advertises
  BLE.setLocalName("Fall_detector");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  switchCharacteristic.setEventHandler(BLERead, switchCharacteristicWritten);
  // set an initial value for the characteristic
  switchCharacteristic.setValue(0);

  // start advertising
  BLE.advertise();

  Serial.println(("Bluetooth device active, waiting for connections..."));
  // initialize the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

}

const int points_number = 300; // 300 points were chosen based on some data visulation and analysis of falls
float val_acc[points_number];     // Array will contain the number of  acceleration points that will be analysed once a threshold is reached
float val_gyr[points_number];     // Array will contain the number of  gyroscope values points that will be analysed once a threshold is reached

float acc_lower_threshold = 0.75;   // Lower acceleration threhold, this value was chosen  based on data visualization of several falls
float acc_higher_threshold = 2.5;   // Higher acceleration threshold : Peak of acceleration chosen based on data visualization of several falls
float gyro_higher_threhold = 160;   // Higher gyroscope threshold : Peak of gyroscope value chosen based on data visualization of several falls
int   free_fall_points_number = 4;  // Data visualization has shown that for a free-fall, at least 4 acceleration points below 0.75g can be observed

void loop() {
  // poll for BLE events
  BLE.poll();
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  float gx, gy, gz;                // Acceleration components that will be read using the Arduino IMU
  float ax,ay,az;                  // Acceleration components that will be read using the Arduino IMU
  
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) { //checking if the accelerometer and gyroscope are available
    IMU.readAcceleration(ax, ay, az); // Reading acceleration components, unit : m/s^2
    IMU.readGyroscope(gx, gy, gz);    // Reading gyroscope value components, unit : degrees per second
    float Acc;
    float Gyro;
    Acc = sqrt(ax*ax+ay*ay+az*az);     // Euclidean norm of the acceleration
    Gyro = sqrt(gx*gx+gy*gy+gz*gz);    // Euclidean norm of gyroscope value


 int count = 0;   // Counts the number of points bellow the acc_lower_threshold value in order to identify a free-fall
 
 if(Acc < acc_lower_threshold){  // At this point, a possible free-fall phase might occur  
    for(int i=0;i<points_number;i++){ 
    IMU.readAcceleration(ax, ay, az); //Taking new measures
    IMU.readGyroscope(gx, gy, gz);    //Taking new measures
    
    val_acc[i]= sqrt(ax*ax+ay*ay+az*az);    // Calculating euclidean norm
    val_gyr[i]= sqrt(gx*gx+gy*gy+gz*gz);
    
    }
    for(int i=0;i<free_fall_points_number;i++){   // Cheking whether a free-fall phase is actually in progress
      if(val_acc[i]> acc_lower_threshold){                        // If one of the measured accelerations is above the set limit, there is no free-fall
        Serial.println("NOT FALL");
        delay(2000);
         break;
     }
     else{
      count++;
     }
      }
     if(count == free_fall_points_number){ // At this stage, the free-fall phase has been succeffuly detected
        for(int i=free_fall_points_number;i<points_number;i++){ 
          if(val_acc[i]> acc_higher_threshold){    // Cheking among the rest of points for an acceleration peak
            for(int j=0;j< points_number;j++){
              if(val_gyr[j]> gyro_higher_threhold){                // Cheking among the rest of points for a gyroscope value peak, 
                  Serial.println("FALL DETECTED");
                  switchCharacteristic.writeValue(1);
                  Serial.println("value 1 written ");
                  //delay(200);
                   
                  break;
              }
             }
             break;
           }
      }
      count = 0;
     } 
    }
else {
    switchCharacteristic.writeValue(0);
    Serial.println("LED off");
    digitalWrite(ledPin, LOW);
  }
    
   }

  
}
