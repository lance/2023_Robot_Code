package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//LEDS
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

//Constants
import frc.robot.Constants.kSensors;

public class Indications extends SubsystemBase {

  private AddressableLED m_LedStrip;
  private AddressableLEDBuffer m_LedData;

  public Indications() {
    m_LedStrip = new AddressableLED(kSensors.ledPort);
    m_LedData = new AddressableLEDBuffer(kSensors.ledLength);
    m_LedStrip.setData(m_LedData);
    m_LedStrip.start();
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
