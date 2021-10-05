/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ColorWheelConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.BallFeedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.XboxControllerButtonMappings;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BallFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final BallFeedSubsystem ballFeedSubsystem = new BallFeedSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final ColorWheelSubsystem colorWheelSubsystem = new ColorWheelSubsystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  //private final PixySubsystem pixySubsystem = new PixySubsystem();

  private final Compressor Comp = new Compressor();

  private final XboxController xboxDriver = new XboxController(0);
  private final XboxController xboxOperator = new XboxController(1);

  // The autonomous command
  private final Command autoCommand = new AutoCommand(shooterSubsystem, drivetrainSubsystem, intakeSubsystem, ballFeedSubsystem);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //new TargetPowerCell(drivetrainSubsystem, pixySubsystem).schedule();

    // Start the cameras
    UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture();
    UsbCamera cam2 = CameraServer.getInstance().startAutomaticCapture();
    cam1.setResolution(160, 120);
    cam2.setResolution(320, 240);

    //    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    // CommandScheduler.getInstance()
    //     .onCommandInitialize(
    //         command ->
    //             Shuffleboard.addEventMarker(
    //                 "Command initialized", command.getName(), EventImportance.kNormal));
    // CommandScheduler.getInstance()
    //     .onCommandInterrupt(
    //         command ->
    //             Shuffleboard.addEventMarker(
    //                 "Command interrupted", command.getName(), EventImportance.kNormal));
    // CommandScheduler.getInstance()
    //     .onCommandFinish(
    //         command ->
    //             Shuffleboard.addEventMarker(
    //                 "Command finished", command.getName(), EventImportance.kNormal));

 // Configure the button bindings
    configureButtonBindings();

    drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
      double speed = xboxDriver.getRawAxis(DriveConstants.kJoystickFwd) - xboxDriver.getRawAxis(DriveConstants.kJoystickRev);
      double rot = xboxDriver.getRawAxis(DriveConstants.kJoystickRotate);

      if (xboxDriver.getRawButton(DriveConstants.kSuperSlowBtn))
      {
        speed *= 0.7;
        rot *= 0.6;
      }

      drivetrainSubsystem.arcadeDrive(speed, rot);

      // Overheat protection
      if (drivetrainSubsystem.driveLeftIsHot()) {
        xboxDriver.setRumble(RumbleType.kLeftRumble, 0.5);
      } else {
        xboxDriver.setRumble(RumbleType.kLeftRumble, 0);
      }

      if (drivetrainSubsystem.driveRightIsHot()) {
        xboxDriver.setRumble(RumbleType.kRightRumble, 0.5);
      } else {
        xboxDriver.setRumble(RumbleType.kRightRumble, 0);
      }
    }, drivetrainSubsystem));
    
    // By default (so basically, not in autonomous), run a command that runs the ballFeed at
    // the speed specified by the joystick axis identified by kBallFeedJoystick (if the joystick
    // is not being moved, the speed is zero, if it is all the way forward, the speed is 1.00, etc.)
    // Motor speed is specified as a decimal between -1.00 and 1.00, where -1.00 is backward 100% speed,
    // and 1.00 is forwards 100% speed.
    ballFeedSubsystem.setDefaultCommand(new RunCommand(() -> 
      ballFeedSubsystem.run(xboxOperator.getRawAxis(BallFeedConstants.kElevatorJoystick), xboxOperator.getRawAxis(BallFeedConstants.kConveyorJoystick)), ballFeedSubsystem));

      intakeSubsystem.setDefaultCommand(new RunCommand(() ->
      intakeSubsystem.run(-xboxDriver.getRawAxis(IntakeConstants.kIntakeJoystick)), intakeSubsystem));

    colorWheelSubsystem.setDefaultCommand(new RunCommand(() -> {
      double colorWheelSpeed = 0;
      
      if (xboxOperator.getPOV() == ColorWheelConstants.kColorWheelLeftPOV)
      {
        colorWheelSpeed = -0.25;
      }
      else if (xboxOperator.getPOV() == ColorWheelConstants.kColorWheelRightPOV)
      {
        colorWheelSpeed = 0.25;
      }

      colorWheelSubsystem.run(colorWheelSpeed);
    }, colorWheelSubsystem));

    climberSubsystem.setDefaultCommand(new RunCommand(() -> {
      double hookSpeed = 0;
      final double HOOK_SPEED_MAX = 0.27;

      if (xboxOperator.getPOV() == 0)
      {
        hookSpeed = HOOK_SPEED_MAX;
      }
      else if (xboxOperator.getPOV() == 180)
      {
        hookSpeed = -HOOK_SPEED_MAX;
      }

      if (xboxOperator.getRawButton(ClimbConstants.kHookMorePower))
      {
        hookSpeed *= 2.4;
      }

      climberSubsystem.runHook(hookSpeed);
    }, climberSubsystem));
  }
  // This is called in the robot's autonomousInit method (when the robot is put into auto)
  public void readyForAuto() {
    readyForTeleop();
  }

  // This is called in the robot's teleopInit method (when the robot is put into teleop).
  public void readyForTeleop() {
    Comp.start();
    colorWheelSubsystem.reset();
    drivetrainSubsystem.reset();
    intakeSubsystem.reset();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Operator buttons
    JoystickButton intakeExtendBtn = new JoystickButton(xboxOperator, IntakeConstants.kExtendBtn);
    JoystickButton colorWheelBtn = new JoystickButton(xboxOperator, ColorWheelConstants.kFlipBtn);
    JoystickButton flyWheelBtn = new JoystickButton(xboxOperator, ShooterConstants.kFlyWheelBtn);
    JoystickButton shooterManualIncRight = new JoystickButton(xboxOperator, ShooterConstants.kRightRotateOneDegreeBtn);
    JoystickButton shooterManualIncLeft = new JoystickButton(xboxOperator, ShooterConstants.kLeftRotateOneDegreeBtn);
    JoystickButton shooterManualIncFiveDegrees = new JoystickButton(xboxOperator, ShooterConstants.kManualRotateButtonsFiveDegreesBtn);
    JoystickButton shooterFlywheelIncSpeed = new JoystickButton(xboxOperator, ShooterConstants.kManualFlywheelIncSpeedBtn);
    JoystickButton shooterFlywheelDecSpeed = new JoystickButton(xboxOperator, ShooterConstants.kManualFlywheelDecSpeedBtn);
    

    intakeExtendBtn.whenPressed(new InstantCommand(() ->
      intakeSubsystem.toggleExtension(), intakeSubsystem));
    
    colorWheelBtn.whenPressed(new InstantCommand(() ->
      colorWheelSubsystem.setFlipped(true), colorWheelSubsystem));
    
    colorWheelBtn.whenReleased(new InstantCommand(() ->
      colorWheelSubsystem.setFlipped(false), colorWheelSubsystem));
    
    flyWheelBtn.whenPressed(new InstantCommand(() ->
      shooterSubsystem.flyWheelSpin(true), shooterSubsystem));

    shooterFlywheelIncSpeed.whenPressed(new InstantCommand(() ->
      shooterSubsystem.manualFlywheelVernier(true), shooterSubsystem));

    shooterFlywheelDecSpeed.whenPressed(new InstantCommand(() ->
      shooterSubsystem.manualFlywheelVernier(false), shooterSubsystem));

    //these buttons add or subtract a position reference vernier to the shooter rotate 
    // The position reference comes from the limelite, so the vernier will also need to be subtracted from the feedback to the limelite,
    //  otherwise it will auto correct the manual vernier back out.
    shooterManualIncRight.whenPressed(new InstantCommand(() ->
      shooterSubsystem.manualRotateVernier(true), shooterSubsystem));

    shooterManualIncLeft.whenPressed(new InstantCommand(() ->
      shooterSubsystem.manualRotateVernier(false), shooterSubsystem));

    //when held, the above vernier buttons have more authority, adding and subtracting 5 degrees of position rather than 1 degree.    
    shooterManualIncFiveDegrees.whenHeld(new InstantCommand(() ->
      shooterSubsystem.manualRotateSpeed(true), shooterSubsystem));

    shooterManualIncFiveDegrees.whenReleased(new InstantCommand(() ->
      shooterSubsystem.manualRotateSpeed(false), shooterSubsystem));

    // Driver buttons
    JoystickButton shifterBtn = new JoystickButton(xboxDriver, DriveConstants.kShiftBtn);
    JoystickButton slowBackBtn = new JoystickButton(xboxDriver, DriveConstants.kSlowBackBtn);
    JoystickButton slowFwdBtn = new JoystickButton(xboxDriver, DriveConstants.kTargetFwdBtn);
    JoystickButton superSlowBtn = new JoystickButton(xboxDriver, DriveConstants.kSuperSlowBtn);
    JoystickButton climbBtn = new JoystickButton(xboxDriver, ClimbConstants.kClimbBtn);
    JoystickButton climbReverseBtn = new JoystickButton(xboxDriver, ClimbConstants.kClimbReverseBtn);

    shifterBtn.whenPressed(new InstantCommand(() ->
      drivetrainSubsystem.shiftDown(), drivetrainSubsystem));
    
    shifterBtn.whenReleased(new InstantCommand(() ->
      drivetrainSubsystem.shiftUp(), drivetrainSubsystem));

    
    slowBackBtn.whileHeld(new InstantCommand(() ->
      drivetrainSubsystem.arcadeDrive(superSlowBtn.get() ? -0.2: -0.4, xboxDriver.getRawAxis(DriveConstants.kJoystickRotate) * 0.8),
      drivetrainSubsystem));

    slowFwdBtn.whileHeld(new InstantCommand(() ->
      drivetrainSubsystem.arcadeDrive(superSlowBtn.get() ? 0.2 : 0.4, xboxDriver.getRawAxis(DriveConstants.kJoystickRotate) * 0.8),
      drivetrainSubsystem));
    

    climbBtn.whenPressed(new InstantCommand(() -> 
      climberSubsystem.runWench(), climberSubsystem));
    
    climbBtn.whenReleased(new InstantCommand(() -> 
      climberSubsystem.stopWench(), climberSubsystem));

    
    climbReverseBtn.whenPressed(new InstantCommand(() ->
      climberSubsystem.reverseWench(), climberSubsystem));
    
    climbReverseBtn.whenReleased(new InstantCommand(() -> 
      climberSubsystem.stopWench(), climberSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
