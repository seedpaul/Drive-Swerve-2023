// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.autonomous.FirstPath;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final XboxController m_driver = new XboxController(Constants.XboxControllerConstants.DRIVER_CONTROLLER);

  private final SwerveDriveSubsystem m_drivetrainSubsystem;
  private SendableChooser<Command> m_TrajectoryChooser;
  private FirstPath firstPath;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrainSubsystem = SwerveDriveSubsystem.getInstance();

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_driver));
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    configureButtonBindings();
    firstPath = new FirstPath(m_drivetrainSubsystem);
    setupTrajectoryDashboardChooser();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_TrajectoryChooser.getSelected();
    
  }
  private void setupTrajectoryDashboardChooser()
  {
    m_TrajectoryChooser = new SendableChooser<Command>();
    m_TrajectoryChooser.setDefaultOption("firstPath", firstPath.getCommand());
    m_TrajectoryChooser.addOption("secondPath", firstPath.getCommand());

    SmartDashboard.putData(m_TrajectoryChooser);
  }
}
