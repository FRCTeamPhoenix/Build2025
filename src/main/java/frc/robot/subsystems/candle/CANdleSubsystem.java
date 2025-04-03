package frc.robot.subsystems.candle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.candle.CANdleIO.CANdleState;
import frc.robot.util.PathfindingUtils;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class CANdleSubsystem extends SubsystemBase {
  private final CANdleIO candle;
  private final CANdleIOInputsAutoLogged inputs = new CANdleIOInputsAutoLogged();
  private final Supplier<Pose2d> poseSupplier;
  private final IntSupplier stateSupplier;
  private final BooleanSupplier hpLight;
  private final BooleanSupplier hasTags;
  private boolean hasEnabled = false;

  private CANdleState[] levelStates = {CANdleState.Blue, CANdleState.Cyan, CANdleState.Green};

  private boolean override = false;

  public CANdleSubsystem(
      CANdleIO io,
      Supplier<Pose2d> poseSupplier,
      IntSupplier stateSupplier,
      BooleanSupplier hpLight,
      BooleanSupplier hasTags) {
    this.candle = io;
    this.poseSupplier = poseSupplier;
    this.stateSupplier = stateSupplier;
    this.hasTags = hasTags;
    this.hpLight = hpLight;
  }

  @Override
  public void periodic() {
    candle.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);
    if (DriverStation.isEStopped()) {
      candle.setMode(CANdleState.Off);
      return;
    }

    if (DriverStation.isTeleopEnabled()) {
      hasEnabled = true;
      if (!override) {
        if (!hpLight.getAsBoolean()) {
          Pose2d reef = PathfindingUtils.getZoneReefPose(poseSupplier.get(), new Transform2d());
          if (reef != poseSupplier.get()) {
            if (poseSupplier
                    .get()
                    .minus(PathfindingUtils.getZoneReefPose(poseSupplier.get(), new Transform2d()))
                    .getTranslation()
                    .getX()
                > FieldConstants.REEF_BUFFER) {
              if (stateSupplier.getAsInt() - 3 < levelStates.length) {
                candle.setMode(levelStates[stateSupplier.getAsInt() - 3]);
              }
            } else {
              candle.setMode(CANdleState.Orange);
            }
          } else {
            candle.setMode(CANdleState.Orange);
          }
        } else {
          candle.setMode(CANdleState.White);
        }
      }
    } else if (DriverStation.isAutonomousEnabled()) {
      hasEnabled = true;
      candle.setMode(CANdleState.RainbowAnimation);
    } else {
      if (hasEnabled) {
        if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
          candle.setMode(CANdleState.RedLarson);
        } else {
          candle.setMode(CANdleState.BlueLarson);
        }
      } else {
        if (hasTags.getAsBoolean()) {
          candle.setMode(CANdleState.White);
        } else {
          if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
            candle.setMode(CANdleState.RedLarson);
          } else {
            candle.setMode(CANdleState.BlueLarson);
          }
        }
      }
    }
  }

  public void setState(CANdleState state) {
    override = true;
    candle.setMode(state);
  }
}
