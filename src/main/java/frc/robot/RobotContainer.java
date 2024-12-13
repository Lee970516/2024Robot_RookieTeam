// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.OutNote;
import frc.robot.commands.OutNote_Down;
import frc.robot.commands.ShootPrepSpeaker;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.ShootSpeaker_Auto;
import frc.robot.commands.ShooterReverse_Index;
import frc.robot.commands.prepPassNote;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WestCoastSubsystem;

import java.util.concurrent.Phaser;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final WestCoastSubsystem m_WestCoastSubsystem = new WestCoastSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverController_ID);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.operatorController_ID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    DoubleSupplier xSpeedFunc = ()-> driverController.getRawAxis(1);
    DoubleSupplier zSpeedFunc = ()-> driverController.getRawAxis(5);

    BooleanSupplier isSlowFunc = ()-> driverController.getHID().getXButton();
    BooleanSupplier ifShoot = ()-> operatorController.getHID().getRightBumper();

    driverController.b().whileTrue(
      Commands.runOnce(()-> {
        m_WestCoastSubsystem.resetGyro();
      })
    );

    operatorController.x().whileTrue(new IntakeNote(m_IntakeSubsystem));
    operatorController.y().whileTrue(new OutNote_Down(m_IntakeSubsystem));
    operatorController.a().whileTrue(new OutNote(m_IntakeSubsystem));
    operatorController.y().whileTrue(new ShooterReverse_Index(m_IntakeSubsystem, m_ShooterSubsystem));
    operatorController.rightTrigger().whileTrue(new ShootSpeaker(m_IntakeSubsystem, m_ShooterSubsystem, ifShoot));
    operatorController.leftTrigger().whileTrue(new ShootSpeaker(m_IntakeSubsystem, m_ShooterSubsystem, ifShoot));


    m_WestCoastSubsystem.setDefaultCommand(new ManualDrive(m_WestCoastSubsystem, xSpeedFunc, zSpeedFunc, isSlowFunc));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new ShootSpeaker_Auto(m_ShooterSubsystem, m_IntakeSubsystem);
  }
}
