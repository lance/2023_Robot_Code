package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;
import java.lang.Math;

/**
  Filter based of the WPILib slewrate limiter. 
  Uses separate limits for the rate of change away from zero and towards zero (accel, decel)
*/
public class SplitSlewRateLimiter {
  private double accelLimit;
  private double decelLimit;
  private double prevVal;
  private double prevTime;

  // Rate limit away from zero, towards zero, and initial value of the filter
  public SplitSlewRateLimiter(double accelLimit, double decelLimit, double initValue){
    this.accelLimit = accelLimit;
    this.decelLimit = decelLimit;
    prevVal = initValue;
    prevTime = WPIUtilJNI.now() * 1e-6;
  }

  // Overload defaulting initial value to zero
  public SplitSlewRateLimiter(double accelLimit, double decelLimit){
    this(accelLimit, decelLimit, 0);
  }

  // Set the value of the filter
  public void setValue(double value){
    prevVal = value;
    prevTime = WPIUtilJNI.now() * 1e-6;
  }

  // Change the limits
  public void setLimits(double accelLimit, double decelLimit){
    this.accelLimit = accelLimit;
    this.decelLimit = decelLimit;
  }

  // Update the filter with a new input value and return the filtered value
  public double calculate(double newValue){
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - prevTime;
    double speedDelta = Math.abs(newValue) - Math.abs(prevVal);

    // Switch between the two based on the sign of the rate of absolute change (speed vs velocity)
    if (speedDelta >= 0) prevVal += MathUtil.clamp(newValue - prevVal, -accelLimit * elapsedTime, accelLimit * elapsedTime);
    else prevVal += MathUtil.clamp(newValue - prevVal, -decelLimit * elapsedTime, decelLimit * elapsedTime);

    prevTime = currentTime;
    return prevVal;
  }

  //Update the filter without limiting rate of change
  public double overrideCalculate(double newValue){
    prevVal = newValue;
    prevTime = WPIUtilJNI.now() * 1e-6;
    return prevVal;
  }
}
