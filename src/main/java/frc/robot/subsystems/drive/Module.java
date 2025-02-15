// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.util.PhoenixUtils.PhoenixFF;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final PhoenixFF driveFeedforward;
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute

  private final Alert driveAlert;
  private final Alert turnAlert;
  private final Alert encoderAlert;


  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    driveAlert = new Alert("Module " + Integer.toString(index) + " Drive Motor is disconnected", AlertType.kError);
    turnAlert = new Alert("Module " + Integer.toString(index) + " Turn Motor is disconnected", AlertType.kError);
    encoderAlert = new Alert("Module " + Integer.toString(index) + " Encoder is disconnected", AlertType.kError);

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.CURRENT_MODE) {
      case REAL:
      case REPLAY:
        driveFeedforward = new PhoenixFF(0.1, 0.13, 0.0);
        driveFeedback = new PIDController(0.05, 0.0, 0.0);
        turnFeedback = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        driveFeedforward = new PhoenixFF(0.0, 0.0, 0.0);
        driveFeedback = new PIDController(0.5, 0.0, 0.0);
        turnFeedback = new PIDController(8, 0.0, 0.0);
        break;
      default:
        driveFeedforward = new PhoenixFF(0.0, 0.0, 0.0);
        driveFeedback = new PIDController(0.0, 0.0, 0.0);
        turnFeedback = new PIDController(0.0, 0.0, 0.0);
        break;
    }

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // On first cycle, reset relative turn encoder
    if (Constants.CURRENT_MODE != Mode.SIM) {
      // Wait until absolute angle is nonzero in case it wasn't initialized yet
      if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
        turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
      }

    } else {
      turnRelativeOffset = Rotation2d.kZero;
    }

    // Run closed loop turn control
    if (angleSetpoint != null) {
      io.setTurnVoltage(
          turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Run drive controller
        double velocityRadPerSec = speedSetpoint / DriveConstants.WHEEL_RADIUS;
        io.setDriveVoltage(
            driveFeedforward.calculate(velocityRadPerSec, 0)
                + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
      }
    }

    driveAlert.set(!inputs.driveConnected);
    turnAlert.set(!inputs.turnConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    state.optimize(getAngle());
    state.cosineScale(getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = state.angle;
    speedSetpoint = state.speedMetersPerSecond;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (turnRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return inputs.turnPosition.plus(turnRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
