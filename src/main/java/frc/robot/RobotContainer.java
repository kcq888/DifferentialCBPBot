/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DiffDriveCommand;
import frc.robot.subsystems.DiffDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.AutoCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Driver and Co-Driver Joysticks definitions
  public final Joystick driverOI=new Joystick(Constants.DRIVEROI);
  public final Joystick coDriverOI=new Joystick(Constants.CODRIVEROI);

  // The robot's subsystems and commands are defined here...
  private final DiffDriveSubsystem diffDriveSubsystem = new DiffDriveSubsystem();
  private final AutoCommand autoCommand = new AutoCommand(diffDriveSubsystem);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    final double xSpeed = -driverOI.getY(GenericHID.Hand.kLeft) * DiffDriveSubsystem.kMaxSpeed;
    final double rotation = -driverOI.getX(GenericHID.Hand.kRight) * DiffDriveSubsystem.kMaxAngularSpeed;
    
    diffDriveSubsystem.setDefaultCommand(
      new RunCommand(()-> diffDriveSubsystem.drive(xSpeed, rotation)));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver button mappings
    // Example: 
    // JoystickButton someButton = new JoystickButton(driverOI, Constants.SOMEBUTTON);
    // someButton.whenPressed(new SomeCommand());

    // Co-Driver button mappings
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An AutoCommand will run in autonomous
    return autoCommand;
  }
}
