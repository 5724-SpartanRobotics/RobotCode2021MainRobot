/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  private CANSparkMax leftMotorMain = new CANSparkMax(DriveConstants.kMotorR1ID, MotorType.kBrushless);
  private CANSparkMax leftMotorAlt1 = new CANSparkMax(DriveConstants.kMotorR2ID, MotorType.kBrushless);
  private CANSparkMax leftMotorAlt2 = new CANSparkMax(DriveConstants.kMotorR3ID, MotorType.kBrushless);
  private CANSparkMax rightMotorMain = new CANSparkMax(DriveConstants.kMotorL1ID, MotorType.kBrushless);
  private CANSparkMax rightMotorAlt1 = new CANSparkMax(DriveConstants.kMotorL2ID, MotorType.kBrushless);
  private CANSparkMax rightMotorAlt2 = new CANSparkMax(DriveConstants.kMotorL3ID, MotorType.kBrushless);

  private final CANPIDController leftPID;
  private final CANPIDController rightPID;
  private final CANEncoder leftEncoder;
  private final CANEncoder rightEncoder;
  
  private DifferentialDrive robotDrive;
  private final DifferentialDriveOdometry odometry;

  private Solenoid shifterSolenoid = new Solenoid(DriveConstants.kShifterSolenoidID);
  private boolean isLowGear = false;
  
  final int MAX_MOTOR_TEMP = 100;

  /**
   * Creates a new ExampleSubsystem.
   */
  public DrivetrainSubsystem() {
    leftPID = leftMotorMain.getPIDController();
    rightPID = rightMotorMain.getPIDController();
    leftPID.setP(DriveConstants.kPDriveVel);
    leftPID.setI(0);
    leftPID.setD(0);
    leftPID.setFF(DriveConstants.kFFDriveVel);
    rightPID.setP(DriveConstants.kPDriveVel);
    rightPID.setI(0);
    rightPID.setD(0);
    rightPID.setFF(DriveConstants.kFFDriveVel);

    leftEncoder = leftMotorMain.getEncoder();
    rightEncoder = rightMotorMain.getEncoder();

    odometry = new DifferentialDriveOdometry(new Rotation2d(0));

    leftMotorMain.setIdleMode(IdleMode.kBrake);
    leftMotorAlt1.setIdleMode(IdleMode.kBrake);
    leftMotorAlt2.setIdleMode(IdleMode.kBrake);
    rightMotorMain.setIdleMode(IdleMode.kBrake);
    rightMotorAlt1.setIdleMode(IdleMode.kBrake);
    rightMotorAlt2.setIdleMode(IdleMode.kBrake);

    leftMotorAlt1.follow(leftMotorMain);
    leftMotorAlt2.follow(leftMotorMain);
    
    rightMotorAlt1.follow(rightMotorMain);
    rightMotorAlt2.follow(rightMotorMain);

    robotDrive = new DifferentialDrive(leftMotorMain, rightMotorMain);
  }

  public void arcadeDrive(double speed, double rotation) {
    robotDrive.arcadeDrive(speed, -rotation * 0.7);
  }

  public void tankDriveVelocity(double leftVel, double rightVel) {
    robotDrive.arcadeDrive(0, 0);
    leftPID.setReference(leftVel * DriveConstants.kMotorRPMPerMetersPerSecond, ControlType.kVelocity);
    //rightPID.setReference(rightVel * DriveConstants.kMotorRPMPerMetersPerSecond, ControlType.kVelocity);
    System.out.println("Actual: " + leftEncoder.getVelocity());
    System.out.println("Set: " + leftVel * DriveConstants.kMotorRPMPerMetersPerSecond);
    //System.out.println(rightVel / DriveConstants.kMetersPerRotation);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    odometry.resetPosition(new Pose2d(0, 0, new Rotation2d()), new Rotation2d());
  }

  public void shift() {
    isLowGear = !isLowGear;
    shifterSolenoid.set(isLowGear);
  }

  public void shiftDown() {
    isLowGear = true;
    shifterSolenoid.set(isLowGear);
  }

  public void shiftUp() {
    isLowGear = false;
    shifterSolenoid.set(isLowGear);
  }

  public void reset() {
    shiftUp();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // TODO make this use the gyro and convert the encoder distances to meters
    double leftPos = leftEncoder.getPosition() / DriveConstants.kMotorRotationsPerMeter;
    double rightPos = rightEncoder.getPosition() / DriveConstants.kMotorRotationsPerMeter;
    double diff = leftPos - rightPos;

    odometry.update(new Rotation2d(0/*diff / DriveConstants.kTrackWidthMeters*/), leftPos, leftPos);
  }

  public boolean driveLeftIsHot() {
    return leftMotorMain.getMotorTemperature() >= MAX_MOTOR_TEMP
        || leftMotorAlt1.getMotorTemperature() >= MAX_MOTOR_TEMP
        || leftMotorAlt2.getMotorTemperature() >= MAX_MOTOR_TEMP;
  }
  public boolean driveRightIsHot() {
    return rightMotorMain.getMotorTemperature() >= MAX_MOTOR_TEMP
        || rightMotorAlt1.getMotorTemperature() >= MAX_MOTOR_TEMP
        || rightMotorAlt2.getMotorTemperature() >= MAX_MOTOR_TEMP;
  }

}
