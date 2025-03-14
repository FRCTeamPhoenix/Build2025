package frc.robot.subsystems.candle;

import org.littletonrobotics.junction.AutoLog;



public interface CANdleIO {
    @AutoLog
    public static class CANdleIOInputs {
      //public double appliedVolts = 0;
      public String mode = "off";
    } 

  
    public default void updateInputs(CANdleIOInputs inputs) {}
  
    public default void setMode(String mode) {}
}

