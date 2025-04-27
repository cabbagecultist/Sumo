// Haven't included Arduino.h here because it's already included in I2CHelper.h (It needs the byte type)
#include "I2CHelper.h"
#include <Wire.h>
#include <Constants.h>


//==============================================================================
// I2C BUS UTILITIES
//==============================================================================

/**
 * Resets the I2C bus if it hangs by:
 * 1. Checking if SDA or SCL lines are stuck low
 * 2. Using special recovery for stuck SDA
 * 3. Sending stop condition and re-initializing the bus
 * 
 * This function uses the mode configured by USE_FAST_I2C_RESET to select between
 * the thorough, reliable reset method or a faster but potentially less robust method.
 */
void I2CHelper::resetI2CBus()
{
  if (USE_FAST_I2C_RESET) {
    resetI2CBusFast();
  } else {
    Serial.println("Resetting I2C bus");
    Wire.end();
  
    // Check if SDA line is stuck low (common failure mode)
    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
  
    bool sclHigh = digitalRead(I2C_SCL_PIN);
    bool sdaHigh = digitalRead(I2C_SDA_PIN);
  
    if (!sclHigh)
    {
      Serial.println("WARNING: SCL line is stuck LOW - severe bus error");
    }
  
    if (!sdaHigh)
    {
      Serial.println("WARNING: SDA line is stuck LOW - attempting special recovery");
  
      // Special recovery for stuck SDA: Force SCL cycles until SDA is released
      pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
      digitalWrite(I2C_SCL_PIN, HIGH);
  
      // Toggle SCL up to 20 times to try to get slave to complete transaction
      for (int i = 0; i < 20 && !digitalRead(I2C_SDA_PIN); i++)
      {
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(I2C_PULSE_DELAY_US);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(I2C_PULSE_DELAY_US);
      }
  
      // If SDA is still low, we have a serious problem
      if (!digitalRead(I2C_SDA_PIN))
      {
        Serial.println("ERROR: Could not clear SDA line. Hardware problem likely!");
      }
      else
      {
        Serial.println("SDA line successfully released");
      }
    }
  
    // Standard bus recovery procedure
    pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);
    pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
  
    // Pull up SCL to ensure it's high (when in open-drain mode)
    digitalWrite(I2C_SCL_PIN, HIGH);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(I2C_PULSE_DELAY_US);
  
    // Toggle SCL multiple times to release stuck devices
    for (int i = 0; i < I2C_CLOCK_PULSE_COUNT; i++)
    {
      digitalWrite(I2C_SCL_PIN, LOW);
      delayMicroseconds(I2C_PULSE_DELAY_US);
      digitalWrite(I2C_SCL_PIN, HIGH);
      delayMicroseconds(I2C_PULSE_DELAY_US);
    }
  
    // Send STOP condition (SDA low->high while SCL is high)
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(I2C_PULSE_DELAY_US);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(I2C_PULSE_DELAY_US);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(I2C_PULSE_DELAY_US * 2);
  
    // Return pins to normal INPUT mode
    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
  
    // Check if the bus is clear now
    if (!digitalRead(I2C_SDA_PIN) || !digitalRead(I2C_SCL_PIN))
    {
      Serial.println("WARNING: I2C bus lines still not HIGH after reset attempt");
    }
    else
    {
      Serial.println("I2C bus lines are both HIGH after reset");
    }
  
    // Reinitialize I2C bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_SPEED_NORMAL);
    delay(100); // Allow time for devices to reset
  
    Serial.println("I2C bus reset complete");
  }
}

void I2CHelper::resetI2CBusFast()
{
  Serial.println("Fast I2C reset...");
  Wire.end();
  
  // Quick check of bus state
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  
  // Fast reset procedure - minimal timing, fewer checks
  pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
  
  // Send 9 clock pulses (the minimum required to ensure the slave releases the bus)
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(1); // Minimal delay
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(1); // Minimal delay
  }
  
  // Quick STOP condition
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delayMicroseconds(2);
  
  // Reinitialize I2C bus immediately
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_SPEED_NORMAL);
  
  Serial.println("Fast I2C reset complete");
}

/**
 * Tests if a device responds at the given I2C address
 * @param address The I2C address to test
 * @return true if the device responds, false otherwise
 */
bool I2CHelper::testI2CAddress(byte address)
{
  Wire.beginTransmission(address);
  byte status = Wire.endTransmission();

  Serial.printf("I2C test at address 0x%02X: ", address);
  printI2CStatus(status);

  return (status == 0);
}

void I2CHelper::printI2CStatus(byte status)
{
  switch (status)
  {
  case 0:
    Serial.println("Success");
    break;
  case 1:
    Serial.println("Data too long");
    break;
  case 2:
    Serial.println("NACK on address (device not connected)");
    break;
  case 3:
    Serial.println("NACK on data");
    break;
  case 4:
    Serial.println("Other error");
    break;
  default:
    Serial.println("Unknown status");
    break;
  }
}