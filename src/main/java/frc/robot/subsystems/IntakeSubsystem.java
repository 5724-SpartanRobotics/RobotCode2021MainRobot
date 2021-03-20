/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private VictorSPX intakeMotor = new VictorSPX(IntakeConstants.kMotorIntakeID);
  private Solenoid armSolenoid = new Solenoid(IntakeConstants.kSolenoidID);
  private boolean intakeExtended = false;

  /**
   * Creates a new ExampleSubsystem.
   */
  public IntakeSubsystem() {
    
  }

  public void toggleExtension() {
    intakeExtended = !intakeExtended;
    armSolenoid.set(intakeExtended);
  }

  public void setExtended(boolean extended) {
    intakeExtended = extended;
    armSolenoid.set(intakeExtended);
  }

  public void reset() {
    intakeExtended = false;
    armSolenoid.set(intakeExtended);
  }

  public void run(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
