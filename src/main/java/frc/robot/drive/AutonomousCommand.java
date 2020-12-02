package frc.robot.drive;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutonomousCommand extends SequentialCommandGroup {
    private static final String trajectoryJSON = "paths/YourPath.wpilib.json";

    public AutonomousCommand(final DriveSubsystem driveSubsystem) throws java.io.IOException {

        final var trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        final var trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

        final var ramseteCommand = new RamseteCommand(trajectory, driveSubsystem::getPose,
                new RamseteController(AutoConstants.ramseteB, AutoConstants.ramseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.driveKinematics, driveSubsystem::getWheelSpeeds,
                new PIDController(DriveConstants.kpDriveVel, 0, 0), new PIDController(DriveConstants.kpDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                driveSubsystem::tankDriveVolts, driveSubsystem);

        final var stopCommand = new InstantCommand(() -> driveSubsystem.tankDriveVolts(0, 0), driveSubsystem);

        // Run path following command, then stop at the end.
        addCommands(ramseteCommand, stopCommand);
    }
}
