// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.SwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** the indexes to address buttons on the controller */
    public static final class ControllerConstants {
        public static final Joystick kDriver = new Joystick(0);
        //public static final Joystick kManipulator = new Joystick(1);

        public static final int kLeftVertical = 1;
        public static final int kRightVertical = 5;
        public static final int kLeftHorizontal = 0;
        public static final int kRightHorizontal = 4;
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;

        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kLeftOptions = 7;
        public static final int kRightOptions = 8;
        public static final int kLeftJoystickPressed = 9;
        public static final int kRightJoystickPressed = 10;
    }

    /** static SwerveModule objects, Can IDs, PID values, ect. */
    public static final class DriveConstants {

        public static final double kMaxRotSpeed = 260; // should be in degrees per second
        public static final double kMaxDriveSpeed = 2.0; // should be in meters per second

        public static final int kIMUid = 13;
        public static final double[] kSwerveSteerPID = {0.015, 0.0, 0.001};//{0.01, 0.0, 0.001};
        public static final double[] kSwerveDrivePID = {0.3, 2, 0.01}; //{0.35, 2,0.01};
        public static final double kSwerveDriveEncConv = 0.8;
        public static final SwerveModule LFSwerve = new SwerveModule(5, 6, 11, kSwerveSteerPID, kSwerveDrivePID, 0.165, 0.165,166.3);
        public static final SwerveModule RFSwerve = new SwerveModule(8, 52, 12, kSwerveSteerPID, kSwerveDrivePID, 0.165, -0.165, 128);
        public static final SwerveModule LRSwerve = new SwerveModule(4, 7, 10, kSwerveSteerPID, kSwerveDrivePID, -0.165, 0.165, 70.1);
        public static final SwerveModule RRSwerve = new SwerveModule(3, 2, 9, kSwerveSteerPID, kSwerveDrivePID, -0.165, -0.165, 7.1);
    }
}