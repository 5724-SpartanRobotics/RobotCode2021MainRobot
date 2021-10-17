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
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterSubsystem extends SubsystemBase {
  public boolean CanShoot = false;
  private CANSparkMax motorFlyWheel = new CANSparkMax(ShooterConstants.kMotorFlyWheel, MotorType.kBrushless);
  private CANSparkMax motorRotate = new CANSparkMax(ShooterConstants.kMotorRotate, MotorType.kBrushless);
  private CANEncoder m_Encoder;
  private CANPIDController m_PidController;
//  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;
  private double _ManualPositionVernier;
  private boolean _ManualPositionVernierHighSpeed;
  private double _RequestedTurretAngle;
  // Degrees btw

  private double _ManualFlywheelPercentSpeedVernier;
  private boolean _FlywheelIsOn;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  /**
   * Creates a new ExampleSubsystem.
   */

  public ShooterSubsystem() {
    motorFlyWheel.setIdleMode(IdleMode.kBrake);
 //   motorRotate.restoreFactoryDefaults();
    m_PidController = motorRotate.getPIDController();
    m_Encoder = motorRotate.getEncoder();
  /*  // PID coefficients
    kP = 3.6e-5;
    kI = 105e-6;
    kD = 0;
    kIz = 0;
    kFF = 0.000156;
    kMaxOutput = 1;
    kMinOutput = -1;
    
    // Smart Motion Coefficients
    maxVel = 250; // rpm  (4 revolutions of the motor is one rev of the rotate gear, which makes about a 45 degree turntable turn. so 24 rpm yields 45 degrees in 10 seconds)
    maxAcc = 500;

    // set PID coefficients
    m_PidController.setP(kP);
    m_PidController.setI(kI);
    m_PidController.setD(kD);
    m_PidController.setIZone(kIz);
    m_PidController.setFF(kFF);
    m_PidController.setOutputRange(kMinOutput, kMaxOutput);

    //
    //  Smart Motion coefficients are set on a CANPIDController object
    //  
    //  - setSmartMotionMaxVelocity() will limit the velocity in RPM of the pid
    //  controller in Smart Motion mode - setSmartMotionMinOutputVelocity() will put
    //  a lower bound in RPM of the pid controller in Smart Motion mode -
    //  setSmartMotionMaxAccel() will limit the acceleration in RPM^2 of the pid
    //  controller in Smart Motion mode - setSmartMotionAllowedClosedLoopError() will
    //  set the max allowed error for the pid controller in Smart Motion mode
    // 
    int smartMotionSlot = 0;
    m_PidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_PidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_PidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_PidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
*/
    // display PID coefficients on SmartDashboard
    /* 
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
    SmartDashboard.putNumber("Set Velocity", 0);
    SmartDashboard.putNumber("Manual Rotate Vernier", 0);
// button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
*/
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Flywheel Speed", 0);
  }
  public void innit(){
    
  }
  public void flyWheelTurnOn(boolean on)
  {
    _FlywheelIsOn = on;
  }
  public void flyWheelToggle(boolean flyWheelOn) {
    _FlywheelIsOn = !_FlywheelIsOn;
    
  }

  public void manualRotateSpeed(boolean high)
  {
    _ManualPositionVernierHighSpeed = high;
  }

  public void manualRotateVernier(boolean right)
  {
    double posAdder;
    if (right)
    {
      if (_ManualPositionVernierHighSpeed)
        posAdder = ShooterConstants.kLargerManualVernier;
      else
        posAdder = ShooterConstants.kSmallManualVernier;
    }
    else
    {
      if (_ManualPositionVernierHighSpeed)
      posAdder = -ShooterConstants.kLargerManualVernier;
    else
      posAdder = -ShooterConstants.kSmallManualVernier;
    }
    double newVal = _ManualPositionVernier + posAdder;
    //clamp the manual vernier because it is added after the setpoint clamp of +/- 4 motor turns
    if (newVal > 2.5)
      newVal = 2.5;
    else if (newVal < -2.5)
      newVal = -2.5;
    _ManualPositionVernier = newVal;
  }

  //If the flywheel is running, increase / descrease the speed setpoint by a vernier
  public void manualFlywheelVernier(boolean inc) {
    if (!_FlywheelIsOn)
      return;
    double speedVernier = _ManualFlywheelPercentSpeedVernier;
    if (inc)
      speedVernier += ShooterConstants.kFlywheelSpeedVernier;
    else
      speedVernier -= ShooterConstants.kFlywheelSpeedVernier;
    //clamp to max/min
    if (speedVernier > 0.5)
      speedVernier = 0.5;
    else if (speedVernier < -0.5)
      speedVernier = -0.5;
    _ManualFlywheelPercentSpeedVernier = speedVernier;
    motorFlyWheel.set(ShooterConstants.kFlywheelSpeed + _ManualFlywheelPercentSpeedVernier);
  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //lime light values
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    CanShoot = y > 10;
    SmartDashboard.putBoolean("Can you shoot ball", CanShoot);
    SmartDashboard.putBoolean("Is Flywheel On", _FlywheelIsOn);
    //flyWheel operation
    if (_FlywheelIsOn) {
      double flyWheelSpeed = speedLmaoBox(y, 8, 35, 1, 2) / 148.6 * 2.15;
      if (flyWheelSpeed > 1)
        flyWheelSpeed = 1;
      SmartDashboard.putNumber("Expected Speed", flyWheelSpeed);
      motorFlyWheel.set(flyWheelSpeed);
    } else {
      motorFlyWheel.set(0);
      SmartDashboard.putNumber("Expected Speed", 0);
    }
    
    //turret rotation
    double processVariable; //turret rotation position in motor turns (4 turns = 45 degrees of turret rotation)
    _RequestedTurretAngle = rotLmaoBox(-1, x, y, 1.5, 2, 8);
    SmartDashboard.putNumber("Requested Angle", _RequestedTurretAngle);
    double setPoint = _RequestedTurretAngle / 45 * 4;
    // clamp the position setpoint to +/- 4 (motor turns)
    if (setPoint > 4)
      setPoint = 4;
    else if (setPoint < -4)
      setPoint = -4;
      //If the turret powered up at an offset from straight forward, then the limelight calculated
      // position will always be off by that amount. This allows the operator left and right bumper
      // buttons to add or subtract 1 degree of rotation.
      setPoint += _ManualPositionVernier;
      // System.out.println(" Position setpoint " + setPoint);
    /**
     * As with other PID modes, Smart Motion is set by calling the setReference
     * method on an existing pid object and setting the control type to kSmartMotion
     */
    //using the position setpoint from the smart dashboard, this will come from limelight code in the future.
    CANError err = m_PidController.setReference(setPoint, ControlType.kSmartMotion);
    SmartDashboard.putNumber("PID reference result", err.value);
    processVariable = m_Encoder.getPosition();

    SmartDashboard.putNumber("SetPoint", setPoint);
  //  SmartDashboard.putNumber("Manual Rotate Vernier", _ManualPositionVernier);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Turret Output", motorRotate.getAppliedOutput());
    SmartDashboard.putNumber("Flywheel Speed", motorFlyWheel.getEncoder().getVelocity());
    
    double area = ta.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  
  }
  // GET GOOD, GET LMAOBOX
  // TF2 reference, this piece of code automatically determines the speed to spin the wheels
  public double speedLmaoBox(double ydeg, double y, double sdeg, double lHeight, double sHeight) {
   double tanY = Math.tan(Math.toRadians(ydeg));
   if (tanY < 0.17)
     tanY = .36;
   double x = (y - lHeight)/tanY;
    double sRad = Math.toRadians(sdeg);
    double xSpeed = Math.sqrt(32*x/(Math.tan(sRad) + (sHeight - y) / x));
    double cosSdeg = Math.cos(sRad);
    if (cosSdeg == 0)
      cosSdeg = .001;
    double spd = xSpeed/cosSdeg;
    //System.out.println(spd);
    return (spd);
  }
  public double rotLmaoBox(double offset, double xdeg, double ydeg, double lHeight, double sHeight, double y) {
    double tanY = Math.tan(Math.toRadians(ydeg));
    if (tanY == 0)
      tanY = .001;
    double d = (y - lHeight)/tanY;
    if (d == 0)
      d = .001;
    double w = d * Math.tan(Math.toRadians(xdeg));
    return Math.toDegrees(Math.atan((w - offset)/d));
  }
}