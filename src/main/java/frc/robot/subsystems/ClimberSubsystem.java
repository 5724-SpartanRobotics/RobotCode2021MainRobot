/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SRamp;
import frc.robot.Constants.ClimbConstants;

public class ClimberSubsystem extends SubsystemBase {
  
  private CANSparkMax hookMotor = new CANSparkMax(ClimbConstants.kHookMotorID, MotorType.kBrushless);
  // The hook accelerated way too quickly, so we added a ramp
  private SRamp hookSpeed = new SRamp();
  private VictorSPX wenchMotorMain = new VictorSPX(ClimbConstants.kWenchMotor1ID);
  private VictorSPX wenchMotorAlt = new VictorSPX(ClimbConstants.kWenchMotor2ID);

  /**
   * Creates a new ExampleSubsystem.
   */
  public ClimberSubsystem() {
    wenchMotorMain.setNeutralMode(NeutralMode.Brake);
    wenchMotorAlt.setNeutralMode(NeutralMode.Brake);

    wenchMotorAlt.follow(wenchMotorMain);

    hookMotor.setIdleMode(IdleMode.kBrake);

    
    hookSpeed.Rate = 0.06;
    hookSpeed.setMaxAccelRate(0.0015);

  }

  public void runHook(double speed) {
    hookSpeed.Setpoint = speed;
  }

  public void runWench() {
    wenchMotorMain.set(ControlMode.PercentOutput, -1.0);
  }

  public void reverseWench() {
    wenchMotorMain.set(ControlMode.PercentOutput, 0.333);
  }

  public void stopWench() {
    wenchMotorMain.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hookSpeed.update();
    hookMotor.set(hookSpeed.getOutput());
  }
}
