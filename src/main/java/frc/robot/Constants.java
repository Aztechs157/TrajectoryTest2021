/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OIConstants {
        public static final int driverControllerPort = 1;
    }

    public static final class DriveConstants {
        public static final int leftMotor1Port = 2;
        public static final int leftMotor2Port = 4;
        public static final int rightMotor1Port = 1;
        public static final int rightMotor2Port = 3;
        public static final boolean leftEncoderReversed = false;
        public static final boolean rightEncoderReversed = true;

        public static final double trackwidthMeters = .611;
        public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(
                trackwidthMeters);

        public static final int encoderCPR = 1024;
        public static final double wheelDiameterMeters = 0.15;
        public static final double encoderPositionConversionFactor =
                // Assumes the encoders are directly mounted on the wheel shafts
                (wheelDiameterMeters * Math.PI) / (double) encoderCPR;

        public static final boolean gyroReversed = true;

        /*
         * These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT! These
         * characterization values MUST be determined either experimentally or
         * theoretically for *your* robot's drive. The Robot Characterization Tool Suite
         * provides a convenient tool for obtaining these values for your robot.
         */
        public static final double ksVolts = 0.132;
        public static final double kvVoltSecondsPerMeter = 2.42;
        public static final double kaVoltSecondsSquaredPerMeter = 0.407;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kpDriveVel = 0.581;
    }

    public static final class AutoConstants {
        // old max speed/acceration = 3
        // public static final double maxSpeedMetersPerSecond = 0.01;
        // public static final double maxAccelerationMetersPerSecondSquared = 0.001;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double ramseteB = 2;
        public static final double ramseteZeta = 0.7;
    }
}
