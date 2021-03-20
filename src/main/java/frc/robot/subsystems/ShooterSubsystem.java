/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax motorFlyWheel = new CANSparkMax(ShooterConstants.kMotorFlyWheel,MotorType.kBrushless);
  private CANSparkMax motorRotate = new CANSparkMax(ShooterConstants.kMotorRotate,MotorType.kBrushless);
  private CANEncoder m_Encoder;
  private CANPIDController m_PidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
/**
   * Creates a new ExampleSubsystem.
   */


  public ShooterSubsystem() {
    motorFlyWheel.setIdleMode(IdleMode.kBrake);
    motorRotate.restoreFactoryDefaults();
    m_PidController = motorRotate.getPIDController();
    m_Encoder = motorRotate.getEncoder();
      // PID coefficients
      kP = 5e-5; 
      kI = 1e-6;
      kD = 0; 
      kIz = 0; 
      kFF = 0.000156; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = 5700;
  
      // Smart Motion Coefficients
      maxVel = 500; // rpm
      maxAcc = 500;
  
      // set PID coefficients
      m_PidController.setP(kP);
      m_PidController.setI(kI);
      m_PidController.setD(kD);
      m_PidController.setIZone(kIz);
      m_PidController.setFF(kFF);
      m_PidController.setOutputRange(kMinOutput, kMaxOutput);
  
     /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_PidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_PidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_PidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_PidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
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
     // read PID coefficients from SmartDashboard
     double p = SmartDashboard.getNumber("P Gain", 0);
     double i = SmartDashboard.getNumber("I Gain", 0);
     double d = SmartDashboard.getNumber("D Gain", 0);
     double iz = SmartDashboard.getNumber("I Zone", 0);
     double ff = SmartDashboard.getNumber("Feed Forward", 0);
     double max = SmartDashboard.getNumber("Max Output", 0);
     double min = SmartDashboard.getNumber("Min Output", 0);
     double maxV = SmartDashboard.getNumber("Max Velocity", 0);
     double minV = SmartDashboard.getNumber("Min Velocity", 0);
     double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
     double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
 
     // if PID coefficients on SmartDashboard have changed, write new values to controller
     if((p != kP)) { m_PidController.setP(p); kP = p; }
     if((i != kI)) { m_PidController.setI(i); kI = i; }
     if((d != kD)) { m_PidController.setD(d); kD = d; }
     if((iz != kIz)) { m_PidController.setIZone(iz); kIz = iz; }
     if((ff != kFF)) { m_PidController.setFF(ff); kFF = ff; }
     if((max != kMaxOutput) || (min != kMinOutput)) { 
       m_PidController.setOutputRange(min, max); 
       kMinOutput = min; kMaxOutput = max; 
       if((maxV != maxVel)) { m_PidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
       if((minV != minVel)) { m_PidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
       if((maxA != maxAcc)) { m_PidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
       if((allE != allowedErr)) { m_PidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
   
       double setPoint, processVariable;
       boolean mode = SmartDashboard.getBoolean("Mode", false);
       if(mode) {
         setPoint = SmartDashboard.getNumber("Set Velocity", 0);
         m_PidController.setReference(setPoint, ControlType.kVelocity);
         processVariable = m_Encoder.getVelocity();
       } else {
         setPoint = SmartDashboard.getNumber("Set Position", 0);
         /**
          * As with other PID modes, Smart Motion is set by calling the
          * setReference method on an existing pid object and setting
          * the control type to kSmartMotion
          */
         m_PidController.setReference(setPoint, ControlType.kSmartMotion);
         processVariable = m_Encoder.getPosition();
       }
       
       SmartDashboard.putNumber("SetPoint", setPoint);
       SmartDashboard.putNumber("Process Variable", processVariable);
       SmartDashboard.putNumber("Output", motorRotate.getAppliedOutput());
   }
  }
}