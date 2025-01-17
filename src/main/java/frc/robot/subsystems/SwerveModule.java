package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.DriveConstants.kSwerveDriveEncConv;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Utils.PID;

public class SwerveModule {

    public final SparkMax steerMotor;
    public final SparkMax driveMotor;
    public final CANcoder enc;
    public final PID steerPID;
    public final  PID drivePID;
    public final double x;
    public final double y;
    public final double encOffs;
    public double angleFromCenter;
    public double distFromCenter;
    public int driveMult = 1;

    /** steering angle in degrees for this module to target relative to the robot frame */
    public double targetSteer = 0;
    /** Wheel velocity in m/s (if kSwerveDriveEncConv is set correctly) for the module to target */
    public double targetDrive = 0;

    
    /**
     * Create a swerve module
     * @param steerID the CAN ID of the steering motor
     * @param driveID the CAN ID of the drive motor
     * @param encID the CAN ID of the cancoder
     * @param pid {P,I,D} as a double array
     * @param drivePID {P,I,D} as a double array
     * @param X x position of module relative to robot center (forward positive)
     * @param Y y position of module relative to robot center (left positive)
     * @param DRVCONV drive encoder conversion value
     * @param ENCOS absolute encoder offset (for centering modules)
     */
    public SwerveModule(int steerID, int driveID, int encID, double[] steerPIDvalue, double[] drivePIDvalue, double X, double Y, double ENCOS) {

        steerMotor = new SparkMax(steerID, MotorType.kBrushless); // create motors and set conversion
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);

        enc = new CANcoder(encID); // this is the steering encoder
        enc.clearStickyFaults();

        steerPID = new PID(steerPIDvalue[0], steerPIDvalue[1], steerPIDvalue[2]);
        drivePID = new PID(drivePIDvalue[0], drivePIDvalue[1], drivePIDvalue[2]);

        x = X; // set the module position values and encoder offsets
        y = Y;
        encOffs = ENCOS;

        calcPosition(0,0);
    }

    /** resets the control PIDs and sets drive motors to zero */
    public void reset() {
        steerPID.reset();
        drivePID.reset();
        steerMotor.set(0);
        driveMotor.set(0);
    }

    /** calculates and sets motor drive powers */
    public void drive() {

        // sets the speed of the steering motor based on the heading error that is calculated
        steerMotor.set(steerPID.calc(getSwerveHeadingError()));

        // sets the drive motor power, the equation is described below
        //  Error = setpoint - PV = [drive power (reversed if needed)] - [Raw motor velocity * Encoder Conversion factor]
        driveMotor.set(drivePID.calc((driveMult * targetDrive) - getMotorSpeed()));

    }

    /** gets the error ranging from -90 to 90 that the swerve drive needs to turn from
     *  <p>
     *  also sets the drive multiplier to account for when it is quicker to drive the motor backwards rather than steering it all the way around.
     */
    private double getSwerveHeadingError() {
        double res = targetSteer - getEncoderPosition();

        // ensure error is within -180 to 180
        while (res < -180) res += 360;
        while (res > 180) res -= 360;

        // if the module would have to travel less distance if it reverses the drive motor, then do that by setting the drivemult = -1
        if (Math.abs(res) > 90) {
            driveMult = -1;
            if (res > 0) res -= 180;
            else res += 180;
        } else driveMult = 1;

        return res;
    }

    /**
     * @return the current angle of the swerve drive
     */
    public double getEncoderPosition() {
        return enc.getAbsolutePosition().getValue().in(Degrees) - encOffs;
    }

    public void calcPosition(double offX, double offY) {
        distFromCenter = Math.sqrt(Math.pow(x + offX, 2)+Math.pow(y + offY, 2)); // These values are required to calculate the vectors needed to make the robot spin
        angleFromCenter = Math.toDegrees(Math.atan2(y + offY, x + offX));           // They represent the position of the swerve module relative to the intended center of rotation
    }

    public double getMotorSpeed() {
        return driveMotor.getEncoder().getVelocity() * kSwerveDriveEncConv;
    }
}