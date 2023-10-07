package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {

    private Drivetrain dt;

    public DriveCommand(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {
        dt.xDriveTarget = 0;
        dt.yDriveTarget = 0;
        dt.rotationTarget = 0;
    }

    @Override
    public void execute() { 
        dt.xDriveTarget = kDriver.getRawAxis(kLeftHorizontal) * kMaxDriveSpeed;
        dt.yDriveTarget = kDriver.getRawAxis(kLeftVertical) * kMaxDriveSpeed;
        dt.rotationTarget = -kDriver.getRawAxis(kRightHorizontal) * kMaxRotSpeed;
        
        // set offsets
        /*
        if (kDriver.getRawButtonPressed(kLeftBumper)) dt.setOffset(-0.65,0);
        else if (kDriver.getRawButtonReleased(kLeftBumper)) dt.setOffset(0,0);
        */

        dt.setOffset(-1 * -kDriver.getRawAxis(kRightVertical), 0);

        if (kDriver.getRawButtonPressed(kRightBumper)) dt.resetFO();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        dt.xDriveTarget = 0;
        dt.yDriveTarget = 0;
        dt.rotationTarget = 0;
    }
    
}
