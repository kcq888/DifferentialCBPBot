/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drive Subsystem PWM PORTS constants
    public static final int LEFT_MASTER = 1;
    public static final int LEFT_FOLLOWER = 2;
    public static final int RIGHT_MASTER = 3;
    public static final int RIGHT_FOLLOWER = 4;

    // Operator Input (OI) - Joystick constants
    public static final int DRIVEROI = 0;
    public static final int CODRIVEROI = 1;
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BACK = 7;
    public static final int START = 8;
    public static final int LEFT_JOYSTICK_X = 0;
  	public static final int LEFT_JOYSTICK_Y = 1;
    public static final int RIGHT_JOYSTICK_X = 4;
    public static final int RIGHT_JOYSTICK_Y = 5;

    // OI Speeds
    public static final double CRAWL = 0.5;
    public static final double FULL = 1.0;
    public static final double SW_DEADBAND = 0.05;

    // Encoders Channels
    public static final int LEFTENCODER_CA = 0;
    public static final int LEFTENCODER_CB = 1;
    public static final int RIGHTENCODER_CA = 2;
    public static final int RIGHTENCODER_CB = 2;

    // Gyro
    public static final int GYRO_PORT = 0;
}
