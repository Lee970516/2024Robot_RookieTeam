// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterReverse_Index extends Command {
  /** Creates a new ShooterReverse_Index. */
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  public ShooterReverse_Index(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeSubsystem = intakeSubsystem;
    this.m_ShooterSubsystem = shooterSubsystem;

    addRequirements(m_IntakeSubsystem, m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.noteBack();
    m_ShooterSubsystem.shooterIndexReverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop();
    m_ShooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeSubsystem.hasNote();
  }
}
