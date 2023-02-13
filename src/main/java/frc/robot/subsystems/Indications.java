package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSensors;

public class Indications extends SubsystemBase {

  public class IndicatorSection {

    public int start;
    public int end;

    public IndicatorSection(int start, int end) {
      this.start = start;
      this.end = end;
    }
  }

  public class RGB {
    public int red;
    public int green;
    public int blue;

    public RGB(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }

  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledData;

  public Indications() {
    ledStrip = new AddressableLED(kSensors.ledPort);
    ledData = new AddressableLEDBuffer(kSensors.ledLength);
    ledStrip.setData(ledData);
    ledStrip.start();
  }

  public void monotone(IndicatorSection subsection, RGB color) {
    for (int i = subsection.start; i <= subsection.end; i++) {
      ledData.setRGB(i, color.red, color.green, color.blue);
    }
  }

  public void alternate(IndicatorSection subsection, float interval, RGB color1, RGB color2) {
    // 2 Colored Pattern

    int patternSize = 2;

    // sets all of the LEDs between the first LED and the last LED indicated.
    for (int i = subsection.start; i <= subsection.end; i++) {
      // color 1
      if (((i + interval) % patternSize) < 1) {
        ledData.setRGB(i, color1.red, color1.green, color1.blue);

      } else {
        // color 2
        ledData.setRGB(i, color2.red, color2.green, color2.blue);
      }
    }
  }

  public void flashing(IndicatorSection subsection, RGB color, int interval) {
    for (int i = subsection.start; i <= subsection.end; i++) {
      // turns the LEDs onn if the interval is one
      if ((interval % 2) == 1) {
        ledData.setRGB(i, color.red, color.green, color.blue);
      } else {
        // sets the LEDs off
        ledData.setRGB(i, 0, 0, 0);
      }
    }
  }

  public void runway(IndicatorSection subsection, RGB color, int interval) {
    // finds many LEDS are in the subsection
    int sectionSize = subsection.end - subsection.start;

    for (int i = subsection.start; i <= subsection.end; i++) {
      // finds where is the sequence the "chasing" LED is
      if ((i % sectionSize) == interval) {
        ledData.setRGB(i, color.red, color.green, color.blue);
      } else {
        ledData.setRGB(i, 0, 0, 0);
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
