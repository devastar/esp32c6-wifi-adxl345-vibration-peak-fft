# ESP32-C6 Zigbee Vibration Sensor Development Notes

This document covers the development of an industrial vibration sensor using an ADXL345 accelerometer, an ESP32-C6 board, and the Arduino Zigbee library. Although the sensor normally reports data via the Temperature Measurement cluster, we repurpose it to monitor motor vibrations. The firmware performs local calibration and FFT-based signal processing so that an external converter in Zigbee2MQTT can re-map the data to a dedicated "vibration" property.

---

## Overview

- **Sensor:** ADXL345 accelerometer (I2C on SDA=19 and SCL=20).
- **Controller:** ESP32-C6 board programmed via the Arduino IDE.
- **Zigbee Endpoint:** Custom endpoint based on ZigbeeTempSensor (renamed to ZigbeeVibrationSensor) that sends vibration data using the Temperature Measurement cluster.
- **Data Processing:** Local calibration determines the baseline acceleration, and FFT processing (using the arduinoFFT library) extracts the dominant vibration frequency.
- **Reporting & Conversion:** The sensor reports the dominant frequency as temperature. An external converter in Zigbee2MQTT re-maps this value to a "vibration" measurement for proper dashboard display.

---

## Hardware Setup & Configuration

### Components

- ESP32-C6 board (with Zigbee support)
- ADXL345 3-axis accelerometer
- Necessary wiring:
  - **VCC** to 3.3V (or as specified by your sensor)
  - **GND** to ground
  - **SDA:** Connect to GPIO19 on the ESP32
  - **SCL:** Connect to GPIO20 on the ESP32

### Wiring Notes

- Double-check voltage requirements between the ESP32 and the ADXL345.
- Secure the sensor in a static position during calibration for optimal accuracy.

---

## 1. Calibration of the ADXL345

Before taking vibration measurements, the ADXL345 sensor must be calibrated to establish the baseline acceleration (typically near 9.81 m/s² due to gravity).

### Calibration Code Example

```cpp
#define FFT_SAMPLES 128
#define SAMPLING_FREQUENCY 500  // in Hz

arduinoFFT FFT = arduinoFFT();

static void vibration_sensor_value_update(void *arg) {
  sensors_event_t event;
  double vReal[FFT_SAMPLES];
  double vImag[FFT_SAMPLES];

  for (;;) {
    // Collect FFT_SAMPLES samples
    for (int i = 0; i < FFT_SAMPLES; i++) {
      if (!accel.getEvent(&event)) {
        Serial.println("Failed to get ADXL345 event!");
        vReal[i] = 0;
      } else {
        double totalAcc = sqrt(event.acceleration.x * event.acceleration.x +
                               event.acceleration.y * event.acceleration.y +
                               event.acceleration.z * event.acceleration.z);
        // Remove the baseline to isolate vibration components
        double sampleValue = totalAcc - calibration_baseline;
        vReal[i] = sampleValue;
      }
      vImag[i] = 0;
      // Delay to achieve the desired sampling frequency
      vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLING_FREQUENCY));
    }

    // Apply a Hamming window before FFT computation
    FFT.Windowing(vReal, FFT_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    // Compute FFT
    FFT.Compute(vReal, vImag, FFT_SAMPLES, FFT_FORWARD);
    // Convert complex numbers to magnitudes
    FFT.ComplexToMagnitude(vReal, vImag, FFT_SAMPLES);

    // Find the greatest amplitude (skipping the DC component at index 0)
    double peak = 0;
    int peakIndex = 1;
    for (int i = 1; i < FFT_SAMPLES / 2; i++) {
      if (vReal[i] > peak) {
        peak = vReal[i];
        peakIndex = i;
      }
    }

    double peakFreq = peakIndex * ((double)SAMPLING_FREQUENCY / FFT_SAMPLES);
    Serial.printf("Peak frequency: %.2f Hz, Amplitude: %.2f\n", peakFreq, peak);

    // Report the dominant frequency as the vibration measurement
    zbVibrationSensor.setVibration(peakFreq);

    // Wait before starting the next FFT sample window
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
```

**Instructions:**

- Ensure that the sensor is static during the calibration process.
- Recalibrate if the sensor's mounting changes or under different environmental conditions.

---

## 2. FFT-Based Vibration Analysis

An FFT (Fast Fourier Transform) is used to extract the dominant vibration frequency from a series of acceleration samples. This frequency is then interpreted as the vibration measurement.

### Requirements

- Install the [arduinoFFT library](https://github.com/kosme/arduinoFFT) via the Arduino Library Manager.
- Define FFT parameters (e.g., 128 samples at a 500 Hz sampling rate).

### FFT Code Example

*The FFT code example is integrated into the calibration code above.*

**Instructions:**

- Adjust `FFT_SAMPLES` and `SAMPLING_FREQUENCY` based on your required resolution and target vibration frequency range.
- Use the Serial Monitor to view FFT output before fully integrating into your Zigbee reporting system.

---

## 3. Using External Converters in Zigbee2MQTT

Since the sensor initially reports through the Temperature Measurement cluster, an external converter in Zigbee2MQTT can re-map this data to a "vibration" property for proper dashboard visualization.

### Steps

1. **Create an External Converter File:**  
   In your Zigbee2MQTT setup, navigate to the `data/external_converters` folder and create a file named `vibration_converter.js`.

2. **Example Converter Code:**

   ```js
   // vibration_converter.js

   const fz = {
     custom_vibration_state: {
       cluster: 'msTemperatureMeasurement',
       type: ['attributeReport', 'readResponse'],
       convert: (model, msg, publish, options, meta) => {
         // Interpret measuredValue (normally temperature) as vibration frequency (Hz)
         let vibValue = Math.round(msg.data.measuredValue);
         return { vibration: vibValue };
       },
     },
   };

   module.exports = { fz };
   ```

3. **Update the Device Definition:**  
   In your Zigbee2MQTT configuration (e.g., `configuration.yaml`), add an entry for your sensor:

   ```yaml
   devices:
     '0x00158d0001abcd12':  # Replace with your sensor's IEEE address
       friendly_name: "Vibration Sensor"
       definition:
         model: "ZigbeeVibrationSensor"
         vendor: "Espressif"
         description: "Custom vibration sensor using FFT analysis, reporting via temperature cluster"
         supports: "vibration"
       fromZigbee:
         - custom_vibration_state
       toZigbee: []
       exposes:
         - type: numeric
           property: vibration
           name: vibration
           unit: "Hz"
   ```

**Instructions:**

- Restart Zigbee2MQTT after adding the external converter file and updating your configuration.
- Edit the converter logic if scaling or offset adjustments are required.

---

## 4. Flashing Instructions

Follow these steps to flash the firmware using the Arduino IDE:

1. **Install the ESP32 by Espressif Boards Library:**  
   - Open **File > Preferences** in the Arduino IDE.
   - Add the URL `https://dl.espressif.com/dl/package_esp32_index.json` in the **Additional Boards Manager URLs** field.
   - Go to **Tools > Board > Boards Manager**, search for *ESP32 by Espressif Systems*, and install the package. (Ensure the installed package version supports ESP32-C6)

2. **Board and Port Setup:**  
   - Under **Tools > Board**, select your ESP32-C6 board.  
   - Choose the correct **Port** for your device.

3. **Set the Zigbee Mode to End Device:**  
   - In **Tools**, confirm that Zigbee mode is set to **End Device**.

4. **Partition Scheme:**  
   - Under **Tools > Partition Scheme**, select **Zigbee 4MB with SPIFFS** for proper Zigbee operation.

5. **Enable USB CDC on Boot:**  
   - Ensure that **USB CDC on Boot** is enabled in the **Tools** menu for reliable serial communication upon startup.

6. **Compile and Upload:**  
   - Click the **Verify** button to compile the sketch. Once the compilation is successful, click the **Upload** button to flash the firmware.

7. **Open the Serial Monitor:**  
   - After flashing, open the Serial Monitor (set to **115200 baud**) to view debugging messages and verify output.

---

## 5. Adjustments & Troubleshooting

- **Calibration:**  
  Recalibrate the sensor if its mounting or environmental conditions change.
- **FFT Parameters:**  
  Adjust the FFT settings (`FFT_SAMPLES` and `SAMPLING_FREQUENCY`) to match your application's resolution and target frequency range.
- **Converter Tweaks:**  
  If the reported vibration data needs scaling or offset adjustment, modify the logic in `vibration_converter.js`.
- **Dashboard Visualization:**  
  Use tools like Grafana to plot the vibration property over time for detailed analysis.
- **Debugging:**  
  Leverage the Serial Monitor to check the calibration and FFT outputs before integrating with the Zigbee network.

---

## Summary

- **Data Processing:** The ESP32-C6 firmware first calibrates the ADXL345 sensor to set a baseline, then collects acceleration data and uses FFT to extract the dominant vibration frequency.
- **Reporting:** The dominant frequency is transmitted via the Temperature Measurement cluster using a custom Zigbee endpoint.
- **Conversion:** An external converter in Zigbee2MQTT re-maps the reported temperature value to a "vibration" measurement.
- **Customization:** All parameters (calibration, FFT, converter logic) can be adjusted as needed for your industrial application.

---

## References

- [ESP32 Zigbee Vibration Sensor Repository](https://github.com/devastar/esp32c6_zigbee-adxl345-vibration/blob/master/README.md)
- [arduinoFFT Library on GitHub](https://github.com/kosme/arduinoFFT)
- [Zigbee2MQTT External Converters Documentation](https://www.zigbee2mqtt.io/converters/external_converters.html)
- [ESP32 Board Setup with Arduino IDE](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)

---

Happy building and good luck with your project!