// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WestCoastConstants;
import frc.robot.subsystems.WestCoastSubsystem;

public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final WestCoastSubsystem m_WestCoastSubsystem;

  private final DoubleSupplier xSpeedFunc;
  private final DoubleSupplier zSpeedFunc;

  private final BooleanSupplier isSlowFunc;

  private double xSpeed;
  private double zSpeed;

  private boolean isSlow;

  public ManualDrive(WestCoastSubsystem westCoastSubsystem, DoubleSupplier xSpeed, DoubleSupplier zSpeed, BooleanSupplier isSlow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_WestCoastSubsystem = westCoastSubsystem;
    
    this.xSpeedFunc = xSpeed;
    this.zSpeedFunc = zSpeed;
    this.isSlowFunc = isSlow;

    addRequirements(m_WestCoastSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xSpeed = xSpeedFunc.getAsDouble();
    zSpeed = zSpeedFunc.getAsDouble();
    isSlow = isSlowFunc.getAsBoolean();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xSpeedFunc.getAsDouble();
    zSpeed = zSpeedFunc.getAsDouble();
    isSlow = isSlowFunc.getAsBoolean();

    if(isSlow) {
      xSpeed = xSpeed*0.4;
      zSpeed = zSpeed*0.4;
    }else {
      xSpeed = xSpeed*0.6;
      zSpeed = zSpeed*0.6;
    }

    m_WestCoastSubsystem.manualDrive(xSpeed, zSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
