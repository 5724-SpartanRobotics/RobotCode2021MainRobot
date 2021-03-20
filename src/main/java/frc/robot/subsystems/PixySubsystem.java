/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;

public class PixySubsystem extends SubsystemBase {
  
  private final Pixy2 pixy;

  /**
   * Creates a new ExampleSubsystem.
   */
  public PixySubsystem() {
    // Set up the pixy
    pixy = Pixy2.createInstance(LinkType.SPI);
    pixy.init();
    pixy.setLamp((byte)1, (byte)1);
    pixy.setLED(255, 255, 255);
  }

  public Pixy2 getPixy() {
      return pixy;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
