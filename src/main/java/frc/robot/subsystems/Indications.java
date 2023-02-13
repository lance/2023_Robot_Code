package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSensors;
import frc.robot.utilities.LEDSubStrip;

public class Indications extends SubsystemBase {

  private AddressableLED armLEDs;
  private AddressableLEDBuffer armLEDsBuffer;
  private LEDSubStrip proximalLeftStrip;

  public Indications() {
    // TODO Add constants for start and end locations and move strip constants out of kSensors
    armLEDs = new AddressableLED(kSensors.ledPort);
    armLEDsBuffer = new AddressableLEDBuffer(kSensors.ledLength);
    proximalLeftStrip = new LEDSubStrip(armLEDsBuffer, 0, 60);
    armLEDs.setData(armLEDsBuffer);
    armLEDs.start();
  }

  public void monotone(LEDSubStrip section, Color color) {
    for (int i = 0; i <= section.getLength(); i++) {
      section.setLED(i, color);
    }
  }

  public void alternate(LEDSubStrip section, float interval, Color color1, Color color2) {
    // 2 Colored Pattern
    int patternSize = 2;

    // sets all of the LEDs between the first LED and the last LED indicated.
    for (int i = 0; i <= section.getLength(); i++) {
      // color 1
      if (((i + interval) % patternSize) < 1) {
        section.setLED(i, color1);

      } else {
        // color 2
        section.setLED(i, color2);
      }
    }
  }

  public void flashing(LEDSubStrip section, Color color, int interval) {
    for (int i = 0; i <= section.getLength(); i++) {
      // turns the LEDs onn if the interval is one
      if ((interval % 2) == 1) {
        section.setLED(i, color);
      } else {
        // sets the LEDs off
        section.setRGB(i, 0, 0, 0);
      }
    }
  }

  public void runway(LEDSubStrip section, Color color, int interval) {
    // finds many LEDS are in the section
    int sectionSize = section.getLength();

    for (int i = 0; i <= section.getLength(); i++) {
      // finds where is the sequence the "chasing" LED is
      if ((i % sectionSize) == interval) {
        armLEDsBuffer.setLED(i, color);
      } else {
        armLEDsBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
