package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class PhoenixUtils {

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   * @return Optimized swerve module state.
   */
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>This is useful for compensating for translational skew when translating and rotating a
   * swerve drivetrain.
   *
   * @param vxMetersPerSecond Forward velocity.
   * @param vyMetersPerSecond Sideways velocity.
   * @param omegaRadiansPerSecond Angular velocity.
   * @param dtSeconds The duration of the timestep the speeds should be applied for.
   * @return Discretized ChassisSpeeds.
   */
  public static ChassisSpeeds discretize(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double dtSeconds) {
    var desiredDeltaPose =
        new Pose2d(
            vxMetersPerSecond * dtSeconds,
            vyMetersPerSecond * dtSeconds,
            new Rotation2d(omegaRadiansPerSecond * dtSeconds));
    var twist = new Pose2d().log(desiredDeltaPose);
    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }

  public static double getDistance(Pose2d poseA, Pose2d poseB) {
    Transform2d poseDelta = poseB.minus(poseA);

    return Math.sqrt(Math.pow(poseDelta.getX(), 2) + Math.pow(poseDelta.getY(), 2));
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
      super(ks, kv, 0);
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
