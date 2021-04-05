/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int kMotorR1ID = 18;
        public static final int kMotorR2ID = 14;
        public static final int kMotorR3ID = 19;
        public static final int kMotorL1ID = 15;
        public static final int kMotorL2ID = 12;
        public static final int kMotorL3ID = 17;

        public static final int kJoystickFwd = XboxControllerButtonMappings.kRightTrigger;
        public static final int kJoystickRev = XboxControllerButtonMappings.kLeftTrigger;
        public static final int kJoystickRotate = XboxControllerButtonMappings.kLeftJoystickXaxis;

        public static final int kShiftBtn = XboxControllerButtonMappings.kBbutton;
        public static final int kSlowBackBtn = XboxControllerButtonMappings.kLeftBumper;
        public static final int kSuperSlowBtn = XboxControllerButtonMappings.kXbutton;
        public static final int kTargetFwdBtn = XboxControllerButtonMappings.kRightBumper;

        public static final int kShifterSolenoidID = 2;

        // All of the stuff below this comment is based on examples from the WPILib trajectory following
        // tutorial (https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/characterizing-drive.html)
        public static final double ksVolts = 0.139;
        public static final double kvVoltSecondPerInch = 1.39;//0.036;
        public static final double kaVoltSecondsSquaredPerInch = 0.451;//0.014;

        public static final double kPDriveVel = .00004;
        public static final double kFFDriveVel = .00000481 * 2;
        public static final double kTrackWidthMeters = 0.6985;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
        
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kMotorRotationsPerMeter = (1 / (Math.PI * 0.1524)) * 5.1;
        public static final double kMotorRPMPerMetersPerSecond = (60.0 / 1.0)  * kMotorRotationsPerMeter;
    }

    public static final class BallFeedConstants {
        public static final int kMotorConveyorID = 9;
        public static final int kMotorElevatorID = 20; 

        public static final int kConveyorJoystick = XboxControllerButtonMappings.kRightJoystickYaxis;
        public static final int kElevatorJoystick = XboxControllerButtonMappings.kLeftJoystickYaxis;

        public static final int kSolenoidID = 0;
    }

    public static final class ShooterConstants {
        public static final int kMotorFlyWheel = 11;
        public static final int kMotorRotate = 16;

        public static final int kFlyWheelBtn = 1;
        public static final int kLeftRotateOneDegreeBtn = XboxControllerButtonMappings.kLeftBumper;
        public static final int kRightRotateOneDegreeBtn = XboxControllerButtonMappings.kRightBumper;
        public static final int kManualRotateButtonsFiveDegreesBtn = XboxControllerButtonMappings.kBbutton;
        public static final int kManualFlywheelIncSpeedBtn = XboxControllerButtonMappings.kStartButton;
        public static final int kManualFlywheelDecSpeedBtn = XboxControllerButtonMappings.kBackButton;

        //8 motor rotations = approximately 90 degrees of travel. These numbers are in motor rotations.
        public static final double kLargerManualVernier = 0.44;
        public static final double kSmallManualVernier = 0.178;
        public static final double kFlywheelSpeed = 0.5;
        public static final double kFlywheelSpeedVernier = 0.1;//10% speed change
    }

    public static final class IntakeConstants {
        public static final int kMotorIntakeID = 32;
        public static final int kSolenoidID = 3;
        public static final int kIntakeJoystick = XboxControllerButtonMappings.kRightJoystickYaxis;
        public static final int kExtendBtn = XboxControllerButtonMappings.kYbutton;
    }

    public static final class ColorWheelConstants {
        public static final int kMotorID = 31;
        public static final int kSolenoidID = 1;
        public static final int kColorWheelLeftPOV = 270;
        public static final int kColorWheelRightPOV = 90;
        public static final int kFlipBtn = XboxControllerButtonMappings.kXbutton;
    }

    public static final class ClimbConstants {
        public static final int kHookMotorID = 1;
        public static final int kWenchMotor1ID = 30;
        public static final int kWenchMotor2ID = 33;

        public static final int kHookUpBtn = XboxControllerButtonMappings.kAbutton;
        public static final int kHookDownBtn = XboxControllerButtonMappings.kAbutton;
        public static final int kHookMorePower = XboxControllerButtonMappings.kLeftJoystick;
        
        public static final int kClimbBtn = XboxControllerButtonMappings.kLeftJoystick;
        public static final int kClimbReverseBtn = XboxControllerButtonMappings.kStartButton;
    }

    public static final class XboxControllerButtonMappings {
        public static final int kAbutton = 1;
        public static final int kBbutton = 2;
        public static final int kXbutton = 3;
        public static final int kYbutton = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kBackButton = 7;
        public static final int kStartButton = 8;
        public static final int kLeftJoystick = 9;
        public static final int kRightJoystick = 10;
        public static final int kLeftJoystickXaxis = 0;
        public static final int kLeftJoystickYaxis = 1;
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;
        public static final int kRightJoystickXaxis = 4;
        public static final int kRightJoystickYaxis = 5;
    }
}
