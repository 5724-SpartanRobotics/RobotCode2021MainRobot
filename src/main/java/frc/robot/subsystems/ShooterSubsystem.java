/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax motorFlyWheel = new CANSparkMax(ShooterConstants.kMotorFlyWheel,MotorType.kBrushless);
  private CANSparkMax motorRotate = new CANSparkMax(ShooterConstants.kMotorRotate,MotorType.kBrushless);
  /**
   * Creates a new ExampleSubsystem.
   */


  public ShooterSubsystem() {
    motorFlyWheel.setIdleMode(IdleMode.kBrake);
  }

  public void flyWheelTurn(boolean isOn) {
    if(isOn == true) {
      motorFlyWheel.set(.5);
    } else {
      motorFlyWheel.set(.0);
    }
  }

  public void rotateTurret () {
    
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
