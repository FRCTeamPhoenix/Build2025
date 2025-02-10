package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class PhoenixUtils {

  public static double getDistance(Pose2d A, Pose2d B) {
    return Math.sqrt(Math.pow(B.getX() - A.getX(), 2) + Math.pow(B.getY() - A.getY(), 2));
  }

  public static double getDistance(Pose3d A, Pose3d B) {
    return Math.sqrt(Math.pow(B.getX() - A.getX(), 2) + Math.pow(B.getY() - A.getY(), 2));
  }

  /* Custom feedforward class */
  public static class PhoenixFF {

    double ks;
    double kv;
    double ka;

    public PhoenixFF(double ks, double kv, double ka) {
      this.ks = ks;
      this.kv = kv;
      this.ka = ka;
      if (kv < 0.0) {
        throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
      }
      if (ka < 0.0) {
        throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
      }
    }

    public double calculate(double velocity, double acceleration) {
      return ks * Math.signum(velocity) + (kv * velocity) + (ka * acceleration);
    }
  }

  /* Extended custom feedforward to add gravity feedforward */
  public static class PhoenixGravFF extends PhoenixFF {

    double kg;

    public PhoenixGravFF(double ks, double kv, double ka, double kg) {
      super(ks, kv, ka);
      this.kg = kg;
    }

    public double calculate(double velocity, double acceleration, double angleRad) {
      return ks * Math.signum(velocity)
          + (kv * velocity)
          + (ka * acceleration)
          + (kg * Math.cos(angleRad));
    }
  }
}
