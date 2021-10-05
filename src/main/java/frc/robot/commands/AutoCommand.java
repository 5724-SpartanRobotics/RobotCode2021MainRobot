package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BallFeedSubsystem;

public class AutoCommand extends SequentialCommandGroup {

    public AutoCommand(ShooterSubsystem shooterSubsystem, DrivetrainSubsystem robotDrive, IntakeSubsystem intakeSubsystem, BallFeedSubsystem ballFeedSubsystem) {
        //The following commands will execute in parallel with different behavior depending on if using
        // the race or the deadline factory methods.
        //  race :  all methods defined in the race are run in parallel, but the first to return interrupts all others
        //          note: RunCommand commands have no ending, so if you use a RunCommand with some wait, only the wait one determines
        //                when the other commands are terminated.
        //  deadline : all methods are run in parallel, but the first method (the deadline), determines when the parallel group ends.
        addCommands(
            new InstantCommand(() -> intakeSubsystem.setExtended(true), intakeSubsystem),
            race(
                new RunCommand(() -> {
                    robotDrive.arcadeDrive(0, 0);
                    ballFeedSubsystem.run(0, 0);
                    robotDrive.arcadeDrive(0.4, 0);
                    shooterSubsystem.flyWheelSpin(true);
                    }, robotDrive, shooterSubsystem, ballFeedSubsystem),
                new WaitUntilCommand(robotDrive.waitForY()),
                new WaitCommand(2) //used for a maximum step timeout
            ),
            race(
                new RunCommand(() -> robotDrive.arcadeDrive(0, 0), robotDrive),
                new WaitCommand(2)
            ),
            race(
                new RunCommand(() -> ballFeedSubsystem.run(-0.2, -0.3),ballFeedSubsystem),
                new WaitCommand(5)//waits until the game clock is at 5 or greater seconds
            ),
            race(
                new RunCommand(() -> {
                    ballFeedSubsystem.run(0,0);
                    intakeSubsystem.setExtended(true);
                    shooterSubsystem.flyWheelSpin(false);
                },  ballFeedSubsystem, intakeSubsystem, shooterSubsystem),
                new WaitCommand(0.2)
            )  
            // new InstantCommand(() -> intakeSubsystem.setExtended(true), intakeSubsystem),
            // race(
            //     new RunCommand(() -> robotDrive.arcadeDrive(-0.4, 0), robotDrive),
            //     new WaitCommand(4)
            // ),
            // race(
            //     new RunCommand(() -> {
            //         robotDrive.arcadeDrive(-0.2, 0);
            //         ballFeedSubsystem.run(1, 1);
            //         intakeSubsystem.run(-0.5);
            //     }, ballFeedSubsystem, intakeSubsystem, robotDrive),
            //     new WaitCommand(1.5)
            // ),
            // race(
            //     new RunCommand(() -> {
            //         robotDrive.arcadeDrive(-0.2, 0);
            //         ballFeedSubsystem.run(-1, -1);
            //         intakeSubsystem.run(-0.5);
            //     }, ballFeedSubsystem, intakeSubsystem, robotDrive),
            //     new WaitCommand(0.25)
            // ),
            // race(
            //     new RunCommand(() -> {
            //         robotDrive.arcadeDrive(-0.2, 0);
            //         ballFeedSubsystem.run(1, 1);
            //         intakeSubsystem.run(-0.5);
            //     }, ballFeedSubsystem, intakeSubsystem, robotDrive),
            //     new WaitCommand(0.5)
            // ),
            // /*new InstantCommand(() -> {
            //     robotDrive.resetEncoders();
            // }, robotDrive),
            // new RamseteCommand(
            //     autoPart1,
            //     robotDrive::getPose,
            //     new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            //     DriveConstants.kDriveKinematics,
            //     robotDrive::tankDriveVelocity,
            //     robotDrive
            // ),
            // new RunCommand(() -> {
            //     robotDrive.tankDriveVelocity(0, 0);
            // })*/
            // race(
            //     new RunCommand(() -> {
            //         ballFeedSubsystem.run(0, 0);
            //         intakeSubsystem.run(0);
            //         robotDrive.arcadeDrive(0.4, -0.4);
            //     }, robotDrive),
            //     new WaitCommand(4.0)
            // )
        );
    }

    private static Trajectory loadTrajectory(String name) {
        String trajectoryJSON = "paths/" + name + ".wpilib.json";

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            return null;
        }
    }

}