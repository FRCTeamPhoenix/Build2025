package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
  @AutoLog
  public static class LimelightIOInputs {
    public boolean connected = false;
    public ObjectObservation[] observations = new ObjectObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record ObjectObservation(double tx, double ty, double ta, Pose3d pose) {
  }

  public default void updateInputs(LimelightIOInputs inputs) {
  }
}
