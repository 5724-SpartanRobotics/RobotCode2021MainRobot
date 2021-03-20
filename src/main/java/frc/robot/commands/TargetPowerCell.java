package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixySubsystem;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class TargetPowerCell extends CommandBase {

    private final DrivetrainSubsystem robotDrive;
    private final PixySubsystem pixySubsystem;
    
    public TargetPowerCell(DrivetrainSubsystem drivebase, PixySubsystem pixy) {
        robotDrive = drivebase;
        pixySubsystem = pixy;
    }

    @Override
    public void execute() {
        Pixy2 pixy = pixySubsystem.getPixy();

        // Gets the number of detected objects with signature 1 (which
        // is calibrated for power cells)
        int count = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);

        double driveSpeed = 0.2;
        double driveRot = 0;

        if (count > 0)
        {
            pixy.getCCC().getBlocks();
            ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
            Block largest = null;

            if (blocks != null)
            {
                for (Block block : blocks)
                {
                    if (block.getSignature() == Pixy2CCC.CCC_SIG1)
                    {
                        if (largest == null || block.getWidth() > largest.getWidth())
                        {
                            largest = block;
                        }
                    }
                }
            }

            if (largest != null)
            {
                SmartDashboard.putNumber("Ball X", largest.getX());
		        SmartDashboard.putNumber("Ball Y", largest.getY());
		        SmartDashboard.putNumber("Ball Angle", largest.getAngle());
            }
        }

        //robotDrive.arcadeDrive(driveSpeed, driveRot);
    }

    @Override
    public boolean isFinished() {
        // false means never finished, so it runs until interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // When the command ends, set the drivetrain to not move (if this were
        // the last command executed, then the drivebase would continue to move
        // until the robot is disabled. Though this should not ever happen, since on
        // this robot the DrivetrainSubsystem has a default command to use the
        // joystick inputs as drivetrain outputs when other commands arn't running,
        // it is best to do this for safety).
        robotDrive.arcadeDrive(0, 0);
    }

}