package frc.robot.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtils {

  /**
   * Minimize the change in heading the desired swerve module state would require
   * by potentially
   * reversing the direction the wheel spins. If this is used with the
   * PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90
   * degrees.
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
   * <p>
   * This function converts a continuous-time chassis speed into a discrete-time
   * one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot
   * moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the
   * x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>
   * This is useful for compensating for translational skew when translating and
   * rotating a
   * swerve drivetrain.
   *
   * @param vxMetersPerSecond     Forward velocity.
   * @param vyMetersPerSecond     Sideways velocity.
   * @param omegaRadiansPerSecond Angular velocity.
   * @param dtSeconds             The duration of the timestep the speeds should
   *                              be applied for.
   * @return Discretized ChassisSpeeds.
   */
  public static ChassisSpeeds discretize(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double dtSeconds) {
    var desiredDeltaPose = new Pose2d(
        vxMetersPerSecond * dtSeconds,
        vyMetersPerSecond * dtSeconds,
        new Rotation2d(omegaRadiansPerSecond * dtSeconds));
    var twist = new Pose2d().log(desiredDeltaPose);
    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative
   * ChassisSpeeds
   * object.
   *
   * @param vxMetersPerSecond     The component of speed in the x direction
   *                              relative to the field.
   *                              Positive x is away from your alliance wall.
   * @param vyMetersPerSecond     The component of speed in the y direction
   *                              relative to the field.
   *                              Positive y is to your left when standing behind
   *                              the alliance wall.
   * @param omegaRadiansPerSecond The angular rate of the robot.
   * @param robotAngle            The angle of the robot as measured by a
   *                              gyroscope. The robot's angle is
   *                              considered to be zero when it is facing directly
   *                              away from your alliance station wall.
   *                              Remember that this should be CCW positive.
   * @return ChassisSpeeds object representing the speeds in the robot's frame of
   *         reference.
   */
  public static ChassisSpeeds fromFieldRelativeSpeeds(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      Rotation2d robotAngle) {
    // CW rotation into chassis frame
    var rotated = new Translation2d(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle.unaryMinus());
    return new ChassisSpeeds(rotated.getX(), rotated.getY(), omegaRadiansPerSecond);
  }

  /*
   * Extended feedforward class to keep calculate using double input
   */
  public static class PhoenixFF extends SimpleMotorFeedforward {

    double ks;
    double kv;

    public PhoenixFF(double ks, double kv) {
      super(ks, kv);
      this.ks = ks;
      this.kv = kv;
    }

    public double simpleCalculate(double velocity) {
      return ks * Math.signum(velocity) + kv * velocity;
    }
  }
}
