package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command {

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

        // use stick axes to set drivetrain targets.
        dt.xDriveTarget = kDriver.getRawAxis(kLeftHorizontal) * kMaxDriveSpeed;
        dt.yDriveTarget = -kDriver.getRawAxis(kLeftVertical) * kMaxDriveSpeed;
        dt.rotationTarget = kDriver.getRawAxis(kRightHorizontal) * kMaxRotSpeed;
        

        // Designate whether or not Field-Orientated control is to be used.
        dt.isFO = !kDriver.getRawButton(kLeftBumper);

        // Set the rotation offset using the vertical axis of the steering stick (this is mostly just fun and as a demonstrator but could have some uses)
        dt.setOffset(-1 * kDriver.getRawAxis(kRightVertical), 0);

        // reset the field orientation 
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
