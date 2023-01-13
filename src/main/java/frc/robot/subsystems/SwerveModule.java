package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Utils.PID;

public class SwerveModule {

    public final CANSparkMax steerMotor;
    public final CANSparkMax driveMotor;
    public final CANCoder enc;
    private final PID steerPID;
    private final  PID drivePID;
    public final double x;
    public final double y;
    public final double encOffs;
    public double angleFromCenter;
    public double distFromCenter;

    public double targetSteer = 0;
    public double targetDrive = 0;

    public int driveMult = 1;


    
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
    public SwerveModule(int steerID, int driveID, int encID, double[] steerPIDvalue, double[] drivePIDvalue, double X, double Y, double DRVCONV, double ENCOS) {

        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless); // create motors and set conversion
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.getEncoder().setPositionConversionFactor(DRVCONV);
        driveMotor.getEncoder().setVelocityConversionFactor(DRVCONV);

        enc = new CANCoder(encID); // this is the steering encoder

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
        steerMotor.set(steerPID.calc(getSwerveHeadingError()));
        //steerMotor.set(getSwerveHeadingError() * 0.005);
        driveMotor.set(drivePID.calc((driveMult * targetDrive) - driveMotor.getEncoder().getVelocity()));
        //driveMotor.set(targetDrive * 0.1 * driveMult);
    }

    /** gets the error ranging from -90 to 90 that the swerve drive needs to turn from
     *  <p>
     *  also sets the drive multiplier to account for when it is quicker to drive the motor backwards rather than steering it all the way around.
     */
    private double getSwerveHeadingError() {
        double res = targetSteer - getEncoderPosition();
        while (res < -180) res += 360; // bring error around to range from -180 to 180
        while (res > 180) res -= 360;
        if (Math.abs(res) > 90) { // if it is quicker to drive motor backwards, set drive multiplier and adjust error
            driveMult = -1;
            if (res > 0) res -= 180;
            else res += 180;
        } else driveMult = 1;
        return res;
    }

    /**
     * 
     * @return the angle of the swerve drive
     */
    public double getEncoderPosition() {
        return enc.getAbsolutePosition() - encOffs;
    }

    public void calcPosition(double offX, double offY) {
        distFromCenter = Math.sqrt(Math.pow(x + offX, 2)+Math.pow(y + offY, 2)); // precalc math for later
        angleFromCenter = Math.toDegrees(Math.atan2(y + offY, x + offX));
    }
}