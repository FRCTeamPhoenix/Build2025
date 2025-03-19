package frc.robot.subsystems.candle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.candle.CANdleIO.CANdleState;
import frc.robot.util.PathfindingUtils;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class CANdleSubsystem extends SubsystemBase {
  private final CANdleIO candle;
  private final CANdleIOInputsAutoLogged inputs = new CANdleIOInputsAutoLogged();
  private final Supplier<Pose2d> poseSupplier;

  private boolean override = false;

  public CANdleSubsystem(CANdleIO io, Supplier<Pose2d> poseSupplier) {
    this.candle = io;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    candle.updateInputs(inputs);
    Logger.processInputs("CANDle", inputs);

    if (!override) {
      Pose2d reef = PathfindingUtils.getZoneReefPose(poseSupplier.get(), new Transform2d());
      if (reef != poseSupplier.get()) {
        if (poseSupplier
                .get()
                .minus(PathfindingUtils.getZoneReefPose(poseSupplier.get(), new Transform2d()))
                .getTranslation()
                .getX()
            > FieldConstants.REEF_BUFFER) {
          candle.setMode(CANdleState.Green);
        } else {
          candle.setMode(CANdleState.Red);
        }
      } else {
        candle.setMode(CANdleState.Orange);
      }
    }
  }

  public void setState(CANdleState state) {
    override = true;
    candle.setMode(state);
  }
}
