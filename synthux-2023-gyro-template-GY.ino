// Title: SynthUX Hackathon 2023 - Daisy Gyro/Accelerometer Template
// Description: Template project for reading Adafruit ICM20X data on Seed
// Hardware: Daisy Seed
// Author: Nick Donaldson
// Resources:
//    - https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu/overview
//    - https://adafruit.github.io/Adafruit_ICM20X/html/index.html

#include <DaisyDuino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// This is a utility class, the file has to be in the same directory
// as the main .ino sketch file.
#include "Smooth.h"

// ===============================
// GLOBAL VARIABLES
// ===============================

// Change this to 0 to disable serial logging
#define LOGGING_ENABLED 0

// This object allows us to configure the Daisy Seed hardware.
static DaisyHardware hw;

// This object allows us to configure and manipulate the gyro/accelerometer sensor.
// You must install the Adafruit ICM20X library from the library manager to use it.
// If you have two sensors you need two of these objects.
static Adafruit_ICM20948 icm;

// Time variables used to determine if enough time has passed to log a sensor reading.
// We can't log data as fast as possible or the system will lock up. You can increase
// the interval to make the logs slower and easier to read but you might miss fast changes
// in the data.
static const uint32_t SENSOR_LOG_INTERVAL = 33;  // milliseconds (33ms = 30 times per second)
static uint32_t last_sensor_log_at = 0;          // milliseconds

// The sensor readings tend to be a little noisy/jumpy for directly
// controlling synthesis parameters. We can smooth them out with this object.
// We will need one of these for each parameter we want to smooth out.
// These are just examples.
SmoothedValue smooth_acc_y;
SmoothedValue smooth_gyro_x, smooth_gyro_z;

// These are some simple audio objects for the example
Oscillator osc, lfo, osc2;
MoogLadder lpf;

// Variables for note changes
// float midi_nn_arr[] = {63.0, 65.0, 66.0, 68.0, 73.0, 75.0, 78.0};
// float midi_nn_arr[] = {39.0, 41.0, 42.0, 44.0, 49.0, 51.0, 54.0};
float midi_nn_arr[] = { 51.0, 53.0, 54.0, 56.0, 61.0, 63.0 };
// static int midi_nn_mid_arr[] = {30.0, 32.0, 34.0, 37.0, 42.0, 44.0, 48.0}; // three octaves lower
static int midi_nn_mid_arr[] = { 39.0, 41.0, 42.0, 44.0, 49.0, 51.0 };  // three octaves lower
// static int midi_nn_mid_arr[] = {47.0, 51.0, 53.0, 54.0, 56.0, 61.0}; // three octaves lower

// static int midi_nn_lower_arr[] = {27.0, 29.0, 30.0, 32.0, 37.0, 39.0, 42.0}; // three octaves lower
// float base_nn = midi_nn_arr[0];
float last_base_nn = midi_nn_arr[0];
float last_base_mid_nn = midi_nn_mid_arr[0];

float gyro_z_threshold = 0.1;

unsigned long prevMillis = 0UL;
unsigned long interval = 1000UL;  // interval to wait for until next note change

// ===============================
// ARDUINO SETUP() AND LOOP()
// ===============================

void setup() {
  float sample_rate;

  // Initialize hardware for Daisy Seed at 48kHz sample rate
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  sample_rate = DAISY.get_samplerate();

// Initialize serial interface for debugging
#if LOGGING_ENABLED
  Serial.begin(115200);
  delay(500);
  Serial.println("Daisy Seed STARTED");
#endif

  // --- Initialize Sensor(s) ---

  // Start the I2C communication with the sensor.
  if (!icm.begin_I2C()) {
#if LOGGING_ENABLED
    Serial.println("[ERROR] Failed to initialize sensor!");
#endif
  }

  // If you have two sensors you need to daisy chain their I2C (SDA/SCL) connections and
  // initialize the other one with another I2C address. To do this you have to bridge the
  // solder jumper on the back of one of the sensors and pass in its alternate I2C address,
  // which if the jumper is bridged will be 0x68 instead of the default 0x69. If you do that
  // it is also a good idea to "pull up" the I2C lines with 47k ohm resistors
  //
  // Something like this should work:
  //
  // icm2.begin_I2C(0x68);
  //
  // You can also use a separate I2C bus on the Seed but that will be more difficult to configure
  // since it requires creating and configuring a new TwoWire() object with different pin numbers.

  // Here you can set the accelerometer sensitity range
  // in units of "earth gravity".
  // You can set it to 2, 4, 8, or 16 G.
  // The sensor will be more sensitive to small changes
  // in acceleration if you use a smaller range.
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);

  // This applies internal filtering to the accelerometer data stream.
  // This is the most possible filtering and tends to reduce the noise the most.
  // If you comment this out or change it the readings will jump around a lot more.
  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);

  // Here you can set the gyroscope sensitity range
  // in units of "degrees per second" (dps)
  // You can set it to 250, 500, 1000, or 2000 dps.
  icm.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS);

  // You can also filter the Gyro data directly from the sensor if you want.
  // I don't know why the library has a typo but it does. (Gyrol instead of Gyro):

  // icm.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_5_7_HZ);

  // This sensor also has a magentometer with
  // a configurable data rate in Hz.
  // You can set it to 10, 20, 50, or 100 Hz, or configure
  // for a single reading at on demand or shut it down completely.
  // NOTE: This one is a little tricky to use because it's scaled based on the magentic
  // field it detects. It can be calibrated to compass north but it will mess
  // up if you're near any magentic fields:

  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

  // NOTE: There are other options for each sensor such as limiting the
  // internal sensor sample rate so readings don't change as often. Currently
  // this code is using the defaults which allows the sensor chip to update data
  // very frequently. Lowering that is mostly to reduce power usage for devices
  // that run on battery.

  // --- Initialize DSP Code ---

  osc.Init(sample_rate);
  osc.SetFreq(220.0f);
  osc.SetWaveform(osc.WAVE_POLYBLEP_TRI);

  osc2.Init(sample_rate);
  osc2.SetFreq(220.0f);
  osc2.SetWaveform(osc.WAVE_SIN);

  lpf.Init(sample_rate);
  lpf.SetFreq(5000.0f);
  lpf.SetRes(0.4f);

  lfo.Init(sample_rate);
  lfo.SetWaveform(osc.WAVE_SIN);

  smooth_acc_y.Init(sample_rate);
  smooth_acc_y.SetSlewMs(200.0f);  // set slew, in milliseconds: higher number = more smoothingless reaction to quick changes

  smooth_gyro_x.Init(sample_rate);
  smooth_gyro_x.SetSlewMs(100.0f);  // set slew, in milliseconds: higher number = more smoothing, less reaction to quick changes

  // GY Edit,
  smooth_gyro_z.Init(sample_rate);
  smooth_gyro_z.SetSlewMs(100.0f);

  // Start Audio
  DAISY.begin(AudioCallback);
}

void loop() {
  // These variables will be our raw readings from the sensor.
  // They are updated every loop but only accessible in the loop() function.
  // This sensor does not have a thermometer but we have to provide a temperature
  // data object to the function that gets sensor data anyway. We can just ignore that one.
  sensors_event_t acc_data;
  sensors_event_t gyro_data;
  sensors_event_t temp_data;
  sensors_event_t magneto_data;

  // Read data from the sensor into the variables
  if (!icm.getEvent(&acc_data, &gyro_data, &temp_data, &magneto_data)) {
#if LOGGING_ENABLED
    Serial.println("[ERROR] Failed to read data from sensor!");
#endif
    delay(500);
    return;
  }

  // Do the above again if you have two sensors

  // Call functions to process the data from each sensor
  processAccelerometer(acc_data);
  processGyroscope(gyro_data);
  processMagnetometer(magneto_data);

  // Serial.print("  Z: ");
  // Serial.print(smooth_gyro_z.Process(), 2);
  // Serial.println();

  float smoothed_gyro_z = smooth_gyro_z.Process();
  Serial.print("  current z: ");
  Serial.print(smoothed_gyro_z, 2);
  Serial.println();

  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis > interval) {
    prevMillis = currentMillis;

    if (smoothed_gyro_z > gyro_z_threshold || smoothed_gyro_z < (0 - gyro_z_threshold)) {
      // int arrLen = sizeof(midi_nn_arr) / sizeof(midi_nn_arr[0]);
      int randomNum = random(6);
      last_base_nn = midi_nn_arr[randomNum];
      last_base_mid_nn = midi_nn_mid_arr[randomNum];
      Serial.print("  random number: ");
      Serial.print(randomNum, 2);
      Serial.println();
    }
  }

  // if (smoothed_gyro_z > gyro_z_threshold || smoothed_gyro_z < (0 - gyro_z_threshold)) {
  //   // int arrLen = sizeof(midi_nn_arr) / sizeof(midi_nn_arr[0]);
  //   int randomNum = random(7);
  //   last_base_nn = midi_nn_arr[randomNum];
  //   Serial.print("  random number: ");
  //   Serial.print(randomNum, 2);
  //   Serial.println();
  // }

  Serial.print("  current note: ");
  Serial.print(last_base_nn, 2);
  Serial.println();

// Log the raw data readings periodically for debugging
#if LOGGING_ENABLED
  // If it's been at least SENSOR_LOG_INTERVAL milliseconds since our last reading, we can
  // log this reading. We can't log too quickly or it will overwhelm the system.
  uint32_t now = millis();
  if (now - last_sensor_log_at > SENSOR_LOG_INTERVAL) {
    // Comment this out and uncomment one of the lines below
    // if you want to use serial plotter to visualize data from
    // one of the sensors instead of logging it as text.
    // !!! Only one line should be uncommented at a time !!!

    logSensorData(acc_data, gyro_data, magneto_data);
    // plotSensorData(acc_data.acceleration);
    // plotSensorData(gyro_data.gyro);
    // plotSensorData(magneto_data.magnetic);

    last_sensor_log_at = now;
  }
#endif
}

// ===============================
// AUDIO CALLBACK
// ===============================

/// The audio callback is where we process/synthesize the audio
void AudioCallback(float **in, float **out, size_t size) {

  for (size_t i = 0; i < size; i++) {
    // Set the filter cutoff using the smoothed y accel value
    // mapped into a range of frequency in Hz that's suitable for our filter.
    // We use the "LOG" mapping because it's an exponential mapping that sounds good for
    // a frequency parameter like filter cutoff.
    float y_norm = smooth_acc_y.Process();
    float cutoff_hz = fmap(y_norm, 100.0f, 10000.0f, Mapping::LOG);

    float lfo_freq = fmap(y_norm, 0.0f, 20.0f);
    lfo.SetFreq(lfo_freq);

    float baseCutoff = 100.0f;  // or whatever you want the default cutoff to be
    float lfoOut = lfo.Process();
    // float lfoOut = 5000 + (lfo.Process() * 5000);

    lpf.SetFreq(baseCutoff * exp2f(lfoOut) * 2.0);

    // LFO controlling the filter cut off point
    lfo.SetFreq(cutoff_hz);

    // Serial.print(base_nn, 2);
    // Serial.println();

    // Base oscillator pitch: MIDI note number 45 which is equal to note A2
    // https://www.inspiredacoustics.com/en/MIDI_note_numbers_and_center_frequencies
    float base_nn = last_base_nn;
    float base_nn_mid = last_base_mid_nn;

    // Modify the oscillator frequency using the smoothed gyro x value
    // Depending on how fast you turn the sensor on its side, this will
    // result in a pitch deviation of 2 semitones from the base pitch
    // in either direction. You can change the 2.0f to another number to
    // make the pitch change by a maximum of that many semitones when the
    // gyro senses rotation.
    base_nn += smooth_gyro_x.Process() * 2.0f;
    base_nn_mid += smooth_gyro_x.Process() * 2.0f;

    // Convert the note number into a frequency in Hz to set the oscillator frequency.
    // The mtof() function takes a note number in and outputs a frequency in Hz.
    osc.SetFreq(mtof(base_nn));
    osc2.SetFreq(mtof(base_nn_mid));

    // Get a sample from the oscillator
    float samp = osc.Process();
    float samp1 = osc2.Process();

    // Process it through the filter
    samp = lpf.Process(samp);
    samp1 = lpf.Process(samp1);

    // Copy to outputs
    out[0][i] = samp;
    out[1][i] = samp;
  }
}

// ===============================
// HELPER FUNCTIONS
// ===============================

void processAccelerometer(sensors_event_t data) {
  // One "earth gravity" is about 9.81 m/s^2 so we should expect
  // the reading to be +/-9.81 if the sensor is tilted on
  // its side in either direction and not being shaken or moved otherwise.
  // We can divide by that value to "normalize" the reading.
  const float ACC_MAX = 9.81f;

  // Raw reading from the Y axis.
  // This is in units of m/s^2.
  float y_raw = data.acceleration.y;

  // We are taking the absolute value of y_raw because in this
  // example we don't care which direction the sensor has been rotated,
  // we only want to know if it's turned on its side.
  //
  // We also limit (clamp) the maximum and minimum value here so that
  // our reading is always in the range 0.0 to 1.0 even if the
  // raw acceleration exceeds 9.81.
  float y_norm = fclamp(fabs(y_raw) / ACC_MAX, 0.0f, 1.0f);

  // Finally we update our smoothed acceleration y object with the value we calculated.
  // Note: it's usually better to set normalized (or at least linear) inputs
  // to the SmoothedValue and then map them to other ranges after the smoothing.
  smooth_acc_y.Set(y_norm);
}

void processGyroscope(sensors_event_t data) {
  // Define the maximum rotation speed we want to measure.
  // The larger this is the faster we have to turn the board
  // to get the reading to max out whatever parameter it controls.
  const float GYRO_MAX = 20.0f;   // degrees per second
  const float GYRO_MAX_Z = 0.2f;  // degrees per second

  // Get a raw reading from the gyroscope (x axis)
  float x_raw = data.gyro.x;
  float z_raw = data.gyro.z;

  // Normalize the reading to -1.0 to 1.0 based on our maximum
  float x_norm = fclamp(x_raw / GYRO_MAX, -1.0f, 1.0f);
  float z_norm = fclamp(z_raw / GYRO_MAX_Z, -1.0f, 1.0f);

  // Set the smoothing target
  smooth_gyro_x.Set(x_norm);
  smooth_gyro_z.Set(z_norm);
}

void processMagnetometer(sensors_event_t data) {
  // Compass data is a little harder to use.
  // It could be helpful for absolute angle measurements if you can calibrate it effectively,
  // but for simplicity's sake it might make more sense to stick with accelerometer/gyroscope.
}

void logSensorData(sensors_event_t acc_data, sensors_event_t gyro_data, sensors_event_t magneto_data) {
  const int log_precision = 2;

  Serial.println();
  Serial.print("[Accelerometer] X: ");
  Serial.print(acc_data.acceleration.x, log_precision);
  Serial.print("  Y: ");
  Serial.print(acc_data.acceleration.y, log_precision);
  Serial.print("  Z: ");
  Serial.print(acc_data.acceleration.z, log_precision);
  Serial.println();

  Serial.print("[Gyroscope]     X: ");
  Serial.print(gyro_data.gyro.x, log_precision);
  Serial.print("  Y: ");
  Serial.print(gyro_data.gyro.y, log_precision);
  Serial.print("  Z: ");
  Serial.print(gyro_data.gyro.z, log_precision);
  Serial.println();

  Serial.print("[Magnetometer]  X: ");
  Serial.print(magneto_data.magnetic.x, log_precision);
  Serial.print("  Y: ");
  Serial.print(magneto_data.magnetic.y, log_precision);
  Serial.print("  Z: ");
  Serial.print(magneto_data.magnetic.z, log_precision);
  Serial.println();
}

void plotSensorData(sensors_vec_t vec) {
  Serial.print("X:");
  Serial.print(vec.x);
  Serial.print(",Y:");
  Serial.print(vec.y);
  Serial.print(",Z:");
  Serial.print(vec.z);
  Serial.println();
}