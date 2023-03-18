package frc.robot.util;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.commands.OTF.GoClosestGrid;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.util.MotionHandler.MotionMode;
import org.littletonrobotics.junction.Logger;

public class FromGridSuggestion {
  private final StringSubscriber sub =
      NetworkTableInstance.getDefault()
          .getStringTopic("/AdvantageKit/RealOutputs/Suggested Node")
          .subscribe("N/A");
  private String memory = "N/A";
  private SuperstructureConfig height;
  private int node = 0;
  private Rotation2d heading = RedHawkUtil.Reflections.reflectIfRed(Rotation2d.fromDegrees(180));
  public Command opSuggestion;
  public Command driveSuggestion;

  public void periodic() {
    String val = sub.get();
    if (!(val == null)) {
      if (memory != val) {
        String level = val.substring(val.lastIndexOf(" | ") + 3);
        String nodeStr = val.substring(0, val.indexOf(" | "));
        try {
          node = Integer.parseInt(nodeStr);
        } catch (NumberFormatException e) {
          RedHawkUtil.ErrHandler.getInstance()
              .addError("Cannot parse node number from FromGridSuggestion");
        }

        Logger.getInstance().recordOutput("FromGridPicker/Level", level);

        switch (level) {
          case "LOWHYBRID":
            height =
                (Robot.gamePieceMode == GamePieceMode.CUBE
                    ? Constants.SuperstructureConstants.SCORE_CUBE_LOW
                    : Constants.SuperstructureConstants.SCORE_CONE_LOW);
            break;
          case "MIDCUBE":
            height = Constants.SuperstructureConstants.SCORE_CUBE_MID;
            break;
          case "HIGHCUBE":
            height = Constants.SuperstructureConstants.SCORE_CUBE_HIGH;
            break;
          case "MIDCONE":
            height = Constants.SuperstructureConstants.SCORE_CONE_MID;
            break;
          case "HIGHCONE":
            height = Constants.SuperstructureConstants.SCORE_CONE_HIGH;
            break;
          default:
            RedHawkUtil.ErrHandler.getInstance()
                .addError("Cannot parse node type from FromGridSuggestion");
            break;
        }
      }
    }

    if (height != null && node != 0) {
      opSuggestion =
          new ParallelCommandGroup(
              Intake.Commands.score(),
              Elevator.Commands.setToHeightAndWait(height),
              FourBar.Commands.setAngleDegAndWait(height));

      driveSuggestion =
          new InstantCommand(
              () -> {
                Robot.motionMode = MotionMode.TRAJECTORY;
                TrajectoryController.getInstance()
                    .changePath(
                        new GoClosestGrid()
                            .regenerateTrajectory(
                                new PathPoint(
                                    RedHawkUtil.Reflections.reflectIfRed(
                                        FieldConstants.Grids.complexLowTranslations[node - 1].plus(
                                            new Translation2d(
                                                Constants.DriveConstants.FieldTunables.GRID_OFFSET,
                                                0))),
                                    heading,
                                    heading,
                                    2))
                            .getTrajectory());
              });
    }
    Logger.getInstance()
        .recordOutput("FromGridPicker/Has operator suggestion", opSuggestion != null);

    Logger.getInstance()
        .recordOutput("FromGridPicker/Has driver suggestion", driveSuggestion != null);
  }
}
