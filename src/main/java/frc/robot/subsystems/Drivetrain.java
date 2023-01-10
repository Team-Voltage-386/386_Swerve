package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;


import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


/** A swervedrive drivetrain */
public class Drivetrain extends SubsystemBase {

        public double xDriveTarget = 0;
        public double yDriveTarget = 0;
        public double rotationTarget = 0;

        private double xPos = 0;
        private double yPos = 0;
        private double angle = 0;

        /** raw heading stored in [0] index */
        private double ypr[] = new double[3];

        public Pigeon2 IMU = new Pigeon2(kIMUid);

        /** flag to handle disable safety */
        private boolean wasEnabled = false;

        // timers for odometry
        private Timer odometryTimer = new Timer();
        private double odoTimerLast = 0;

        // array with modules, update to match robot
        public SwerveModule[] modules = {RFSwerve, RRSwerve, LRSwerve, LFSwerve};


        public Drivetrain() {
                this.init();
        }

        public void init() { 
                odometryTimer.start();
                odoTimerLast = odometryTimer.get();
        }

        @Override
        public void periodic() {
                updateOdometry();
                if (Robot.inst.isEnabled()) {
                        for (SwerveModule swerve : modules) {

                                if (Math.abs(xDriveTarget) > 0.05 || Math.abs(yDriveTarget) > 0.05 || Math.abs(rotationTarget) > 1) {
                                        double angleRad = Math.toRadians(angle); // get angle in radians
                                        // fist add translation
                                        double x = xDriveTarget;
                                        double y = yDriveTarget;
                                        
                                        // then add rotation
                                        double r = ((2*Math.PI*swerve.distFromCenter)/360)*rotationTarget; // rotation speed (should be m/s of module I think)
                                        double rAngle = swerve.angleFromCenter + angle + 90;
                                        x += r*Math.cos(Math.toRadians(rAngle));
                                        y += r*Math.sin(Math.toRadians(rAngle));

                                        // apply vectors to the rotation of the robot (making it field-oriented)
                                        double xFin = (x * Math.cos(angleRad)) + (y * Math.sin(angleRad)); 
                                        double yFin = (x * Math.cos(angleRad + (Math.PI/2))) + (y * Math.sin(angleRad + (Math.PI/2)));

                                        //extract angle and power
                                        swerve.targetSteer = Math.toDegrees(Math.atan2(yFin, xFin));
                                        swerve.targetDrive = Math.sqrt(Math.pow(xFin, 2) + Math.pow(yFin, 2));
                                } else {
                                        swerve.targetDrive = 0;
                                        swerve.targetSteer = swerve.angleFromCenter;
                                }

                                swerve.drive(); 
                                //System.out.println("Drove");
                        }
                        wasEnabled = true;
                } else {
                        if (wasEnabled) for (SwerveModule swerve : modules) swerve.reset();
                        wasEnabled = false;
                }

                updateWidgets();
        }

        public double getRawHeading() {
                double y = ypr[0];
                while (y < 0) y += 360;
                while (y > 360) y -= 360;
                return -y;
        }


        public void setOffset(double offX, double offY) {
                for (SwerveModule swerve : modules) swerve.calcPosition(offX, offY);
        }  

        public void resetFO() {
                IMU.setYaw(0);
        }

        private void updateOdometry() {
                IMU.getYawPitchRoll(ypr);
                angle = getRawHeading();

                if (Robot.inst.isEnabled()) {

                        double time = odometryTimer.get(); // calculate delta T
                        odometryTimer.reset();
                        odometryTimer.start();
                        double deltaT = time-odoTimerLast;
                        odoTimerLast = time;

                        double xAdd = 0;
                        double yAdd = 0;
                        
                        for (SwerveModule swerve : modules) {
                                double aRad = Math.toRadians(angle+swerve.getEncoderPosition());
                                double vel = swerve.driveMotor.getEncoder().getVelocity();
                                xAdd += deltaT * (Math.cos(aRad) * vel);
                                yAdd += deltaT * (Math.sin(aRad) * vel);
                        }

                        xPos += xAdd/modules.length;
                        yPos += yAdd/modules.length;
                }

        }


        private static final ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");
        private static final GenericEntry RFEncoderWidget = driveTab.add("Right Front Encoder", 0).withPosition(1,0).withSize(1,1).getEntry();
        private static final GenericEntry RREncoderWidget = driveTab.add("Right Rear Encoder", 0).withPosition(1,1).withSize(1,1).getEntry();
        private static final GenericEntry LREncoderWidget = driveTab.add("Left Rear Encoder", 0).withPosition(0,1).withSize(1,1).getEntry();
        private static final GenericEntry LFEncoderWidget = driveTab.add("Left Front Encoder", 0).withPosition(0,0).withSize(1,1).getEntry();
        private static final GenericEntry RFVelWidget = driveTab.add("RF Velocity", 0).withPosition(1,2).withSize(1,1).getEntry();
        private static final GenericEntry RRVelWidget = driveTab.add("RR Velocity", 0).withPosition(1,3).withSize(1,1).getEntry();
        private static final GenericEntry LFVelWidget = driveTab.add("LF Velocity", 0).withPosition(0,2).withSize(1,1).getEntry();
        private static final GenericEntry LRVelWidget = driveTab.add("LR Velocity", 0).withPosition(0,3).withSize(1,1).getEntry();
        private static final GenericEntry xPosWidget = driveTab.add("X", 0).withPosition(3,0).withSize(1,1).getEntry();
        private static final GenericEntry yPosWidget = driveTab.add("Y", 0).withPosition(4,0).withSize(1,1).getEntry();
        private void updateWidgets() {
                RFEncoderWidget.setDouble(RFSwerve.getEncoderPosition());
                RREncoderWidget.setDouble(RRSwerve.getEncoderPosition());
                LREncoderWidget.setDouble(LRSwerve.getEncoderPosition());
                LFEncoderWidget.setDouble(LFSwerve.getEncoderPosition());

                RFVelWidget.setDouble(RFSwerve.driveMotor.getEncoder().getVelocity());
                RRVelWidget.setDouble(RRSwerve.driveMotor.getEncoder().getVelocity());
                LFVelWidget.setDouble(LFSwerve.driveMotor.getEncoder().getVelocity());
                LRVelWidget.setDouble(LRSwerve.driveMotor.getEncoder().getVelocity());

                xPosWidget.setDouble(xPos);
                yPosWidget.setDouble(yPos);
        }

}