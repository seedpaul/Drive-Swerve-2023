// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.MasterController;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSequences extends SequentialCommandGroup {

  private final AutoPaths autoPaths;
  private final SwerveDriveSubsystem m_drivetrainSubsystem;
  private final MasterController m_MasterController;

  /** Creates a new AutoSequences. */
  public AutoSequences(SwerveDriveSubsystem in_drivetrainSubsystem, MasterController in_MasterController) {

    m_drivetrainSubsystem = in_drivetrainSubsystem;
    m_MasterController = in_MasterController;

    autoPaths = new AutoPaths(m_drivetrainSubsystem);
    addCommands();
  }

  public Command BLUE_CenterPath() {

    SequentialCommandGroup BLUE_CenterPath = new SequentialCommandGroup(
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new WaitCommand(2.1),
        new InstantCommand(m_MasterController::cubeInConeOut, m_MasterController),
        new WaitCommand(0.1),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new ParallelCommandGroup(autoPaths.BLUE_CenterPath(),
            new SequentialCommandGroup(new WaitCommand(0.25),
                new InstantCommand(m_MasterController::autoCubefloorPickUp, m_MasterController),
                new WaitCommand(4.25),
                new InstantCommand(m_MasterController::gotoMiddle, m_MasterController),
                new InstantCommand(m_MasterController::wristTop, m_MasterController))),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new InstantCommand(m_MasterController::wristZero, m_MasterController),
        new ParallelCommandGroup(
            new SequentialCommandGroup(new WaitCommand(0.825), autoPaths.BLUE_CenterFinishPath()),
            new SequentialCommandGroup(new WaitCommand(1.45),
                new InstantCommand(m_MasterController::cubeOutConeIn, m_MasterController))));

    BLUE_CenterPath.setName("BLUE_CENTER_PATH");
    return BLUE_CenterPath;
  }

  public Command BLUE_SidePath() {

    SequentialCommandGroup BLUE_SidePath = new SequentialCommandGroup(
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new WaitCommand(2.1),
        new InstantCommand(m_MasterController::cubeInConeOut, m_MasterController),
        new WaitCommand(0.1),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new ParallelCommandGroup(autoPaths.BLUE_SidePath(),
            new SequentialCommandGroup(new WaitCommand(1.4),
                new InstantCommand(m_MasterController::autoCubefloorPickUp, m_MasterController),
                new WaitCommand(3.3),
                new InstantCommand(m_MasterController::gotoMiddle, m_MasterController),
                new InstantCommand(m_MasterController::wristTop, m_MasterController))),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new InstantCommand(m_MasterController::wristZero, m_MasterController),
        new ParallelCommandGroup(
            new SequentialCommandGroup(new WaitCommand(0.825), autoPaths.BLUE_SideFinishPath()),
            new SequentialCommandGroup(new WaitCommand(1.45),
                new InstantCommand(m_MasterController::cubeOutConeIn, m_MasterController))));

    BLUE_SidePath.setName("BLUE_SIDE_PATH");
    return BLUE_SidePath;
  }

  public Command BLUE_AutoBalance() {

    SequentialCommandGroup BLUE_AutoBalance = new SequentialCommandGroup(
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new WaitCommand(2.1),
        new InstantCommand(m_MasterController::cubeInConeOut, m_MasterController),
        new WaitCommand(0.1),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new InstantCommand(m_MasterController::zero, m_MasterController),
        autoPaths.BLUE_AutoBalancePath(),
        new WaitCommand(0.5),
        m_drivetrainSubsystem.autoBalance());

    BLUE_AutoBalance.setName("BLUE_AUTO_BALANCE");
    return BLUE_AutoBalance;
  }

  public Command RED_AutoBalance() {

    SequentialCommandGroup RED_AutoBalance = new SequentialCommandGroup(
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new WaitCommand(2.1),
        new InstantCommand(m_MasterController::cubeInConeOut, m_MasterController),
        new WaitCommand(0.1),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new InstantCommand(m_MasterController::zero, m_MasterController),
        autoPaths.RED_AutoBalancePath(),
        new WaitCommand(0.5),
        m_drivetrainSubsystem.autoBalance());

    RED_AutoBalance.setName("RED_AUTO_BALANCE");
    return RED_AutoBalance;
  }

  public Command RED_CenterPath() {

    SequentialCommandGroup RED_CenterPath = new SequentialCommandGroup(
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new WaitCommand(2.1),
        new InstantCommand(m_MasterController::cubeInConeOut, m_MasterController),
        new WaitCommand(0.1),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new ParallelCommandGroup(autoPaths.RED_CenterPath(),
            new SequentialCommandGroup(new WaitCommand(0.25),
                new InstantCommand(m_MasterController::autoCubefloorPickUp, m_MasterController),
                new WaitCommand(4.5),
                new InstantCommand(m_MasterController::gotoMiddle, m_MasterController),
                new InstantCommand(m_MasterController::wristTop, m_MasterController))),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new InstantCommand(m_MasterController::wristZero, m_MasterController),
        new ParallelCommandGroup(
            new SequentialCommandGroup(new WaitCommand(0.825), autoPaths.RED_CenterFinishPath()),
            new SequentialCommandGroup(new WaitCommand(1.45),
                new InstantCommand(m_MasterController::cubeOutConeIn, m_MasterController))));

    RED_CenterPath.setName("RED_CENTER_PATH");
    return RED_CenterPath;
  }

  public Command RED_SidePath() {

    SequentialCommandGroup RED_SidePath = new SequentialCommandGroup(
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new WaitCommand(2.1),
        new InstantCommand(m_MasterController::cubeInConeOut, m_MasterController),
        new WaitCommand(0.1),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new ParallelCommandGroup(autoPaths.RED_SidePath(),
            new SequentialCommandGroup(new WaitCommand(0.25),
                new InstantCommand(m_MasterController::autoCubefloorPickUp, m_MasterController),
                new WaitCommand(4.5),
                new InstantCommand(m_MasterController::gotoMiddle, m_MasterController),
                new InstantCommand(m_MasterController::wristTop, m_MasterController))),
        new InstantCommand(m_MasterController::intakeStop, m_MasterController),
        new InstantCommand(m_MasterController::gotoTop, m_MasterController),
        new InstantCommand(m_MasterController::wristZero, m_MasterController),
        new ParallelCommandGroup(
            new SequentialCommandGroup(new WaitCommand(0.825), autoPaths.RED_SideFinishPath()),
            new SequentialCommandGroup(new WaitCommand(1.45),
                new InstantCommand(m_MasterController::cubeOutConeIn, m_MasterController))));

    RED_SidePath.setName("RED_SIDE_PATH");
    return RED_SidePath;
  }

}
