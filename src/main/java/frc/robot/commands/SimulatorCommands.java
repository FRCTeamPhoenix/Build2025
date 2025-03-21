package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class SimulatorCommands {

  public static Command dropCoral(
      SwerveDriveSimulation sim, DoubleSupplier height, DoubleSupplier angle) {
    return Commands.runOnce(
        () ->
            SimulatedArena.getInstance()
                .addGamePieceProjectile(
                    new ReefscapeCoralOnFly(
                        sim.getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(0.41, 0),
                        sim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        sim.getSimulatedDriveTrainPose().getRotation(),
                        Meters.of(height.getAsDouble() + ElevatorConstants.MIN_HEIGHT),
                        MetersPerSecond.of(2),
                        Radians.of(angle.getAsDouble()))));
  }
}
