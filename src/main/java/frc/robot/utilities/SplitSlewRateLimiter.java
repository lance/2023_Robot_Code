package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;
import java.lang.Math;

/**
  Filter based of the WPILib slewrate limiter. 
  Uses separate limits for the rate of change away from zero and towards zero (accel, decel)
*/
public class SplitSlewRateLimiter {
  private double m_accelLimit;
  private double m_decelLimit;
  private double m_prevVal;
  private double m_prevTime;

  // Rate limit away from zero, towards zero, and initial value of the filter
  public SplitSlewRateLimiter(double accelLimit, double decelLimit, double initValue){
    m_accelLimit = accelLimit;
    m_decelLimit = decelLimit;
    m_prevVal = initValue;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }

  // Overload defaulting initial value to zero
  public SplitSlewRateLimiter(double accelLimit, double decelLimit){
    this(accelLimit, decelLimit, 0);
  }

  // Set the value of the filter
  public void setValue(double value){
    m_prevVal = value;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }

  // Change the limits
  public void setLimits(double accelLimit, double decelLimit){
    m_accelLimit = accelLimit;
    m_decelLimit = decelLimit;
  }

  // Update the filter with a new input value and return the filtered value
  public double calculate(double newValue){
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double speedDelta = Math.abs(newValue) - Math.abs(m_prevVal);

    // Switch between the two based on the sign of the rate of absolute change (speed vs velocity)
    if (speedDelta >= 0) m_prevVal += MathUtil.clamp(newValue - m_prevVal, -m_accelLimit * elapsedTime, m_accelLimit * elapsedTime);
    else m_prevVal += MathUtil.clamp(newValue - m_prevVal, -m_decelLimit * elapsedTime, m_decelLimit * elapsedTime);

    m_prevTime = currentTime;
    return m_prevVal;
  }

  //Update the filter without limiting rate of change
  public double overrideCalculate(double newValue){
    m_prevVal = newValue;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
    return m_prevVal;
  }
}
