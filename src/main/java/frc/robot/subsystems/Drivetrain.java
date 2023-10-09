package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/** A swervedrive drivetrain */
public class Drivetrain extends SubsystemBase {

        public double xDriveTarget = 0;
        public double yDriveTarget = 0;
        public double rotationTarget = 0;

        public double xPos = 0;
        public double yPos = 0;
        public double angle = 0;

        /** raw heading stored in [0] index */
        private double ypr[] = new double[3];

        public Pigeon2 IMU = new Pigeon2(kIMUid);

        /** flag to handle disable safety */
        private boolean wasEnabled = false;

        private final Translation2d m_frontLeftLocation = new Translation2d(
                DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
                DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
        private final Translation2d m_frontRightLocation = new Translation2d(
                DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
                DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);
        private final Translation2d m_backLeftLocation = new Translation2d(
                DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
                DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
        private final Translation2d m_backRightLocation = new Translation2d(
                DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
                DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);

        /**
         * The order that you initialize these is important! Later uses of functions
         * like toSwerveModuleStates will return the same order that these are provided.
         * See
         * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
         */
        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);


        // array with modules, update to match robot
        public SwerveModule[] modules = {FRSwerve, BRSwerve, BLSwerve, FLSwerve};

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                FLSwerve.getSwerveModulePosition(), FRSwerve.getSwerveModulePosition(),
                BLSwerve.getSwerveModulePosition(), BRSwerve.getSwerveModulePosition()
                };

        SwerveModuleState[] moduleStates = new SwerveModuleState[] {
                FLSwerve.getSwerveModuleState(), FRSwerve.getSwerveModuleState(),
                BLSwerve.getSwerveModuleState(), BRSwerve.getSwerveModuleState()
        };

        ChassisSpeeds m_chassisSpeeds = m_kinematics.toChassisSpeeds(moduleStates);

        Pose2d roboPose = new Pose2d();

        SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
                        m_kinematics, gyroGetRotation2D(),
                        modulePositions, 
                        roboPose);

        public Drivetrain() {
                this.init();
        }

        public void init() { 
        }

        @Override
        public void periodic() {
                updateOdometry();
                updateWidgets();
                updateModulePositions();
                updateModuleStates();
                setModuleStates();
        }

        public void setModuleStates() {
                if (Robot.inst.isTeleopEnabled()) {
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
                                        swerve.drivePID.reset();
                                        swerve.targetSteer = swerve.angleFromCenter;
                                }

                                swerve.drive(); 
                        }
                        wasEnabled = true;
                } else {
                        if (wasEnabled && !Robot.inst.isAutonomous()) for (SwerveModule swerve : modules) swerve.reset();
                        wasEnabled = false;
                }
        }
        
        public void setModuleStates(SwerveModuleState[] swerveModules) {
                for (int i = 0; i < swerveModules.length; i++) {
                        modules[i].setModuleState(swerveModules[i]);
                }
        }

        public void updateModuleStates() {
                moduleStates = new SwerveModuleState[] {
                        FLSwerve.getSwerveModuleState(), FRSwerve.getSwerveModuleState(),
                        BLSwerve.getSwerveModuleState(), BRSwerve.getSwerveModuleState()
                };
        }

        public double getRawHeading() {
                double y = ypr[0];
                while (y < 0) y += 360;
                while (y > 360) y -= 360;
                return y;
        }


        public void setOffset(double offX, double offY) {
                for (SwerveModule swerve : modules) swerve.calcPosition(offX, offY);
        }  

        public void resetFO() {
                IMU.setYaw(0);
        }

        /**
         * Returns the yaw of the gyro in Rotation2d format
         * @return orientation of robot chasis (assuming the gyro is mounted facing the front)
         */
        public Rotation2d gyroGetRotation2D() {
                return new Rotation2d(Math.toRadians(ypr[0]));
        }

        private void updateOdometry() {
                IMU.getYawPitchRoll(ypr);
                angle = getRawHeading();

                if (Robot.inst.isEnabled()) {

                        // Update the pose
                        roboPose = m_odometry.update(gyroGetRotation2D(), modulePositions);

                        xPos = roboPose.getX();
                        yPos = roboPose.getY();
                }
        }

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        public void updateModulePositions() {
                modulePositions = new SwerveModulePosition[] {
                        FLSwerve.getSwerveModulePosition(), FRSwerve.getSwerveModulePosition(),
                        BLSwerve.getSwerveModulePosition(), BRSwerve.getSwerveModulePosition()};
        }
       
        public void resetOdometry(Pose2d pose) {
                m_odometry.resetPosition(gyroGetRotation2D(), new SwerveModulePosition[] {}, pose);
        }

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
        // https://github.com/HighlanderRobotics/Rapid-React/blob/main/src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java
        public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
                return new SequentialCommandGroup(
                        new InstantCommand(() -> {
                        // Reset odometry for the first path you run during auto
                        if(isFirstPath){
                                this.resetOdometry(traj.getInitialHolonomicPose());
                        }
                        }),
                        new PPSwerveControllerCommand(
                                traj, 
                                () -> m_odometry.getPoseMeters(), // Pose supplier
                                this.m_kinematics, // SwerveDriveKinematics
                                new PIDController(0.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                                new PIDController(0.5, 0, 0), // Y controller (usually the same values as X controller)
                                new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                                (SwerveModuleState[] states) -> { // Consumes the module states to set the modules moving in the directions we want
                                        m_chassisSpeeds = m_kinematics.toChassisSpeeds(states);
                                        setModuleStates(states);
                                }, // Module states consumer
                                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                                this // Requires this drive subsystem
                        )
                );
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
                RFEncoderWidget.setDouble(FRSwerve.getEncoderPosition());
                RREncoderWidget.setDouble(BRSwerve.getEncoderPosition());
                LREncoderWidget.setDouble(BLSwerve.getEncoderPosition());
                LFEncoderWidget.setDouble(FLSwerve.getEncoderPosition());

                RFVelWidget.setDouble(FRSwerve.driveMotor.getEncoder().getVelocity());
                RRVelWidget.setDouble(BRSwerve.driveMotor.getEncoder().getVelocity());
                LFVelWidget.setDouble(FLSwerve.driveMotor.getEncoder().getVelocity());
                LRVelWidget.setDouble(BLSwerve.driveMotor.getEncoder().getVelocity());

                xPosWidget.setDouble(xPos);
                yPosWidget.setDouble(yPos);
        }

}