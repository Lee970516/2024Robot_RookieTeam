// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PassNote extends Command {
  /** Creates a new PassNote. */
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;

  private final BooleanSupplier ifShootFunc;

  private boolean ifShoot;
  public PassNote(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, BooleanSupplier ifShoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ShooterSubsystem = shooterSubsystem;
    this.m_IntakeSubsystem = intakeSubsystem;

    this.ifShootFunc = ifShoot;

    addRequirements(m_ShooterSubsystem, m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.passNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifShoot = ifShootFunc.getAsBoolean();
    if(m_ShooterSubsystem.ifSpeedArrive_PassNote()) {
      m_IntakeSubsystem.outNote();
    }else {
      m_IntakeSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stopShooter();
    m_IntakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
