/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2021 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallFeedConstants;

public class BallFeedSubsystem extends SubsystemBase {

  private CANSparkMax motorBottom = new CANSparkMax(BallFeedConstants.kMotorConveyorID, MotorType.kBrushless);
  private CANSparkMax motorTop = new CANSparkMax(BallFeedConstants.kMotorElevatorID, MotorType.kBrushless);
  /**
   * Creates a new ExampleSubsystem.
   */
  public BallFeedSubsystem() {
    motorBottom.setIdleMode(IdleMode.kBrake);    
    motorTop.setIdleMode(IdleMode.kBrake);
  }

  public void runTop(double powerTop) {
    System.out.print("Run top ball feed command at power " + powerTop);
    // Deadband at plus or minus 5% (because otherwise the ballFeed drifts)
    if (Math.abs(powerTop) < 0.05)
    {
      powerTop = 0;
    }

    motorTop.set(powerTop);
  }

  public void runBottom(double powerBottom){
    System.out.print("Run bottom ball feed command at power " + powerBottom);
    if (Math.abs(powerBottom) < 0.05)
    {
      powerBottom = 0;
    }

    motorBottom.set(powerBottom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
