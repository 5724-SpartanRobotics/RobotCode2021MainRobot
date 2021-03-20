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

        public static final int kJoystickFwd = 3;
        public static final int kJoystickRev = 2;
        public static final int kJoystickRotate = 0;

        public static final int kShiftBtn = 1;
        public static final int kSlowBackBtn = 5;
        public static final int kSuperSlowBtn = 3;
        public static final int kTargetFwdBtn = 6;

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

        public static final int kConveyorJoystick = 5;
        public static final int kElevatorJoystick = 1;

        public static final int kSolenoidID = 0;
    }

    public static final class ShooterConstants {
        public static final int kMotorFlyWheel = 11;
        public static final int kMotorRotate = 16;

        public static final int kFlyWheelBtn = 1;
        public static final int kRotateLeftBtn = 5;
        public static final int kRotateRightBtn =  6;
    }

    public static final class IntakeConstants {
        public static final int kMotorIntakeID = 32;
        public static final int kSolenoidID = 3;
        public static final int kIntakeJoystick = 5;
        public static final int kExtendBtn = 3;
    }

    public static final class ColorWheelConstants {
        public static final int kMotorID = 31;
        public static final int kSolenoidID = 1;
        public static final int kColorWheelLeftPOV = 270;
        public static final int kColorWheelRightPOV = 90;
        public static final int kFlipBtn = 2;
    }

    public static final class ClimbConstants {
        public static final int kHookMotorID = 16;
        public static final int kWenchMotor1ID = 30;
        public static final int kWenchMotor2ID = 33;

        public static final int kHookUpBtn = 0;
        public static final int kHookDownBtn = 0;
        public static final int kHookMorePower = 8;
        
        public static final int kClimbBtn = 8;
        public static final int kClimbReverseBtn = 7;
    }
}
