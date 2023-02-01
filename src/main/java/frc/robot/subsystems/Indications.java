package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


//LEDS
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;


//Constants
import frc.robot.Constants.kSensors;


public class Indications extends SubsystemBase {

 public class RGB{
   public int red;
   public int green;
   public int blue;




   public RGB(int red, int green, int blue){
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


 public void pattern2(RGB color1,RGB color2, float colorSpeed){
   // 2 Colored Pattern
   int patternSize = 2;

    int offset = 0;
    for (var i = 1; i < ledData.getLength(); i++) {
    //color 1
     if (((i + offset) % patternSize) < 1) {
     ledData.setRGB(i, color1.red, color1.green, color1.blue);

     } else {
     //color 2
     ledData.setRGB(i, color2.red, color2.green, color2.blue);

     }
   }
   //offset to "move" the pattern
   offset += colorSpeed;
   offset %= patternSize;


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
