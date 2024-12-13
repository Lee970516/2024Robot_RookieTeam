// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeWheel;
  private final CANSparkMax intakeArm;

  private final CANcoder armAbsolutedEncoder;
  private final CANcoderConfiguration absolutedEncoderConfig;

  private final RelativeEncoder armEncoder;

  private final PIDController armPid;

  private final ArmFeedforward armFeedforward;

  private double armAngle;

  private final DigitalInput irSensor;

  private double pidOutput;
  private double feedforwardOutput;
  private double output;
  public IntakeSubsystem() {
    intakeWheel = new TalonFX(IntakeConstants.intakeWheel_ID);
    intakeArm = new CANSparkMax(0, MotorType.kBrushless);

    intakeWheel.setNeutralMode(NeutralModeValue.Brake);
    intakeArm.setIdleMode(IdleMode.kBrake);

    intakeWheel.setInverted(IntakeConstants.intakeWheelReverse);
    intakeArm.setInverted(IntakeConstants.intakeArmReverse);

    intakeArm.burnFlash();

    armPid = new PIDController(IntakeConstants.armPid_Kp, IntakeConstants.armPid_Ki, IntakeConstants.armPid_Kd);
    armPid.setIntegratorRange(-0.4, 0.4);


    armFeedforward = new ArmFeedforward(IntakeConstants.armFeedforward_Ks, IntakeConstants.armFeedforward_Kg, IntakeConstants.armFeedforward_Kv);

    armEncoder = intakeArm.getEncoder();

    armAbsolutedEncoder = new CANcoder(IntakeConstants.armAbsolutedEncoder_ID);
    absolutedEncoderConfig = new CANcoderConfiguration();

    absolutedEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absolutedEncoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.absolutedEncoderOffset;

    armAbsolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);

    irSensor = new DigitalInput(IntakeConstants.irSensor_ID);

  }

  public void intake() {
    intakeWheel.setVoltage(IntakeConstants.intakeVol);
  }

  public void noteBack() {
    intakeWheel.setVoltage(IntakeConstants.noteBackVol);
  }

  public void outNote() {
    intakeWheel.setVoltage(IntakeConstants.outNoteVol);
  }

  public void stop() {
    intakeWheel.setVoltage(0);
  }

  public void shootAMP() {
    intakeWheel.setVoltage(IntakeConstants.shootAMPVol);
  }

  public void setDownAngle() {
    armAngle = IntakeConstants.downAngle;
  }

  public void setUpAngle() {
    armAngle = IntakeConstants.upAngle;
  }

  public void setAMPAngle() {
    armAngle = IntakeConstants.AMPAngle;
  }

  public double getAngle() {
    return armAbsolutedEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public double getRadian() {
    return Units.degreesToRadians(armAbsolutedEncoder.getAbsolutePosition().getValueAsDouble()*360);
  }

  public double getAbsolutePosition() {
    return armAbsolutedEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getVelocity() {
    return armEncoder.getVelocity();
  }

  public boolean hasNote() {
    return !irSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidOutput = armPid.calculate(getAngle(), armAngle);
    feedforwardOutput = armFeedforward.calculate(getRadian(), getVelocity())/12;
    if(feedforwardOutput >= 0.4) {
      feedforwardOutput = 0.4;
    }
    output = feedforwardOutput + pidOutput;
    SmartDashboard.putBoolean("Intake/HasNote", hasNote());
    SmartDashboard.putNumber("Intake/pidOutput", pidOutput);
    SmartDashboard.putNumber("Intake/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("Intake/Output", output);
  }
}
