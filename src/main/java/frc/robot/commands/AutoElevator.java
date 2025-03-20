package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.PathfindingUtils;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class AutoElevator extends Command {

  private final Supplier<Pose2d> pose;
  private final Superstructure superstructure;
  private final IntSupplier state;
  private final BooleanSupplier manualOverride;

  public AutoElevator(
      Supplier<Pose2d> position,
      Superstructure superstructure,
      IntSupplier state,
      BooleanSupplier override) {
    this.pose = position;
    this.superstructure = superstructure;
    this.state = state;
    this.manualOverride = override;
    super.addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    if (pose.get()
                .minus(PathfindingUtils.getZoneReefPose(pose.get(), new Transform2d()))
                .getTranslation()
                .getX()
            > FieldConstants.REEF_BUFFER
        && !manualOverride.getAsBoolean()) {
      superstructure.setState(state);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
