// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final TalonFX shooterMotorRight;
  private final TalonFX shooterMotorLeft;
  private final TalonFX shooterMotorIndex;

  private boolean ifSpeedArrive;
  public ShooterSubsystem() {
    shooterMotorRight = new TalonFX(ShooterConstants.shooterMotorRight_ID);
    shooterMotorLeft = new TalonFX(ShooterConstants.shooterMotorLeft_ID);
    shooterMotorIndex = new TalonFX(ShooterConstants.shooterMotorIndex_ID);

    shooterMotorRight.setInverted(ShooterConstants.shooterRightReverse);
    shooterMotorLeft.setInverted(ShooterConstants.shooterLeftReverse);
    shooterMotorIndex.setInverted(ShooterConstants.shooterIndexReverse);

    shooterMotorRight.setNeutralMode(NeutralModeValue.Brake);
    shooterMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    shooterMotorIndex.setNeutralMode(NeutralModeValue.Brake);

    ifSpeedArrive = false;
  }

  public double getShooterRightSpeed() {
    return shooterMotorRight.getVelocity().getValueAsDouble();
  }

  public double getShooterLeftSpeed() {
    return shooterMotorLeft.getVelocity().getValueAsDouble();
  }

  public double getShooterIndexSpeed() {
    return shooterMotorIndex.getVelocity().getValueAsDouble();
  }

  public void shootSpeaker() {
    shooterMotorRight.setVoltage(ShooterConstants.shootSpeakerVol);
    shooterMotorLeft.setVoltage(ShooterConstants.shootSpeakerVol);
    shooterMotorIndex.setVoltage(ShooterConstants.shootSpeakerVol);
  }

  public void passNote() {
    shooterMotorRight.setVoltage(ShooterConstants.passNoteVol);
    shooterMotorLeft.setVoltage(ShooterConstants.passNoteVol);
    shooterMotorIndex.setVoltage(ShooterConstants.passNoteVol);
  }

  public void stopShooter() {
    shooterMotorRight.setVoltage(0);
    shooterMotorIndex.setVoltage(0);
    shooterMotorLeft.setVoltage(0);
  }

  public void shooterIndexReverse() {
    shooterMotorIndex.setVoltage(0);
  }

  public boolean ifSpeedArrive_Speaker() {
    ifSpeedArrive = false;
    if(getShooterRightSpeed() >= ShooterConstants.speakerSpeed_Right
        && getShooterLeftSpeed() >= ShooterConstants.speakerSpeed_Left
        && getShooterIndexSpeed() >= ShooterConstants.speakerSpeed_Index) {
      ifSpeedArrive = true;
    }
    return ifSpeedArrive;
  }

  public boolean ifSpeedArrive_PassNote() {
    ifSpeedArrive = false;
    if(getShooterRightSpeed() >= ShooterConstants.passNoteSpeed_Right
    && getShooterLeftSpeed() >= ShooterConstants.passNoteSpeed_Left
    && getShooterIndexSpeed() >= ShooterConstants.passNoteSpeed_Index) {
      ifSpeedArrive = true;
    }
    return ifSpeedArrive;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/RightShooterVelocity", getShooterRightSpeed());
    SmartDashboard.putNumber("Shooter/LeftShooterVelocity", getShooterLeftSpeed());
    SmartDashboard.putNumber("Shooter/IndexShooterVelocity", getShooterIndexSpeed());
  }
}
