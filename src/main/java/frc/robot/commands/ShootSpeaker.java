// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command {
  /** Creates a new ShootSpeaker. */
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;

  private final BooleanSupplier ifShootFunc;

  private boolean ifShoot;

  public ShootSpeaker(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, BooleanSupplier ifShoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeSubsystem = intakeSubsystem;
    this.m_ShooterSubsystem = shooterSubsystem;
    this.ifShootFunc = ifShoot;

    addRequirements(m_IntakeSubsystem, m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.shootSpeaker();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifShoot = ifShootFunc.getAsBoolean();
    if(m_ShooterSubsystem.ifSpeedArrive_Speaker() && ifShoot) {
      m_IntakeSubsystem.outNote();
      m_ShooterSubsystem.shootSpeaker();
    }else {
      m_IntakeSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop();
    m_ShooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}