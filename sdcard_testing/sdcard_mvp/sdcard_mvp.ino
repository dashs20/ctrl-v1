#include <SD.h>

int chipSelect = 10;

void setup() {
  Serial.begin(9600);

  // SD card initialization
  if (!SD.begin(chipSelect)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");
  blackbox();
}

void loop() {
}

void blackbox(){
    SD.remove("data.txt");
    File dataFile = SD.open("data.txt", FILE_WRITE);

    int myInts[] = {1,2,3,4,5};
    int myInts2[] = {6,7,8,9,10};
    if (dataFile) {
    for(int i = 0; i < sizeof(myInts)/sizeof(myInts[0]); i++){
        dataFile.print(String(myInts[i]) + ",");
        dataFile.println(String(myInts2[i]));
    }
    dataFile.close();
    Serial.println("Data written to SD card.");
    } else {
      Serial.println("Error opening data file.");
    }
    dataFile.close();
    Serial.println("done");
    Serial.println(sizeof(myInts)/sizeof(myInts[0]));

}
