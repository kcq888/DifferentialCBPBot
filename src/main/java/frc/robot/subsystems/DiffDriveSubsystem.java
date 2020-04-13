/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;

public class DiffDriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  // Speed Controllers fields
  private final Talon leftMaster = new Talon(Constants.LEFT_MASTER);
  private final Talon leftFollowr = new Talon(Constants.LEFT_FOLLOWER);
  private final Talon rightMaster = new Talon(Constants.RIGHT_MASTER);
  private final Talon rightFollower = new Talon(Constants.RIGHT_FOLLOWER);
  SpeedControllerGroup leftControllerGroup = new SpeedControllerGroup(leftMaster, leftFollowr);
  SpeedControllerGroup rightControllerGroup = new SpeedControllerGroup(rightMaster, rightFollower);

  // Encoder fields
  private final Encoder leftEncoder = new Encoder(Constants.LEFTENCODER_CA, Constants.LEFTENCODER_CB);
  private final Encoder rightEncoder = new Encoder(Constants.RIGHTENCODER_CA, Constants.RIGHTENCODER_CB);

  // Gyro
  private final AnalogGyro gyro = new AnalogGyro(Constants.GYRO_PORT);

  // PID
  private final PIDController leftPIDController = new PIDController(1, 0, 0);
  private final PIDController rightPIDController = new PIDController(1, 0, 0);

  // Kinematics
  private final DifferentialDriveKinematics kinematics
  = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry odometry;

  /**
   * Constructor - Creates a new DiffDriveSubsystem.
   */
  public DiffDriveSubsystem() {
    gyro.reset();

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    leftEncoder.reset();
    rightEncoder.reset();

    odometry = new DifferentialDriveOdometry(getAngle());
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   * 
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * setSpeeds - sets the desired speed of the Differential Drive system
   * 
   * @param lspeeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftOutput = leftPIDController.calculate(leftEncoder.getRate(),
    speeds.leftMetersPerSecond);
    double rightOutput = rightPIDController.calculate(rightEncoder.getRate(),
    speeds.rightMetersPerSecond);
    leftControllerGroup.set(leftOutput);
    rightControllerGroup.set(rightOutput);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

    /**
   * Updates the field-relative position.
   */
  public void updateOdometry() {
    odometry.update(getAngle(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }
}
