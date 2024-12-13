// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.OdometryThread;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.proto.DifferentialDriveWheelVoltagesProto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.proto.DifferentialDriveKinematicsProto;
import edu.wpi.first.math.kinematics.struct.DifferentialDriveKinematicsStruct;
import edu.wpi.first.math.kinematics.struct.DifferentialDriveWheelSpeedsStruct;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WestCoastConstants;

public class WestCoastSubsystem extends SubsystemBase {
  /** Creates a new WestCoastSubsystem. */
  private final WestCoastModule rightModule;
  private final WestCoastModule leftModule;

  private final DifferentialDrive drive;

  private final Pigeon2 gyro;
  private final Pigeon2Configuration gyroConfig;

  // private final DifferentialDriveOdometry odometry;

  public WestCoastSubsystem() {

    rightModule = new WestCoastModule(
      WestCoastConstants.rightForwardModule_ID
    , WestCoastConstants.rightBackModule_ID
    , WestCoastConstants.rightAbsolutedEncoder_ID
    , WestCoastConstants.rightOffSet
    , WestCoastConstants.rightForwardModuleReserve
    , WestCoastConstants.rightBackModuleReserve
    );

    leftModule = new WestCoastModule(
        WestCoastConstants.leftForwardModule_ID
      , WestCoastConstants.leftBackModule_ID
      , WestCoastConstants.leftAbsolutedEncoder_ID
      , WestCoastConstants.leftOffSet
      , WestCoastConstants.leftForwardModuleReserve
      , WestCoastConstants.leftBackModuleReserve
      );

      gyro = new Pigeon2(WestCoastConstants.gyro_ID);
      gyroConfig = new Pigeon2Configuration();
      gyroConfig.MountPose.MountPoseYaw = -10;
      gyro.getConfigurator().apply(gyroConfig);

      drive = new DifferentialDrive(leftModule.getForwardModule(), rightModule.getForwardModule());

      // odometry = new DifferentialDriveOdometry(getRotation2d(), leftModule.getForwardPosition(), rightModule.getForwardPosition());


      // AutoBuilder.configureLTV(
      //         this::getPose, // Robot pose supplier
      //         this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
      //         this::getSpeeds, // Current ChassisSpeeds supplier
      //         this::Drive_Auto, // Method that will drive the robot given ChassisSpeeds
      //         0.02, // Robot control loop period in seconds. Default is 0.02
      //         new ReplanningConfig(), // Default path replanning config. See the API for the options here
      //         () -> {
      //           // Boolean supplier that controls when the path will be mirrored for the red alliance
      //           // This will flip the path being followed to the red side of the field.
      //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      //           var alliance = DriverStation.getAlliance();
      //           if (alliance.isPresent()) {
      //             return alliance.get() == DriverStation.Alliance.Red;
      //           }
      //           return false;
      //         },
      //         this // Reference to this subsystem to set requirements
      // );
    }

  public void manualDrive(double move, double turn) {
    drive.curvatureDrive(move, turn, false);
  }

  // public void Drive_Auto(ChassisSpeeds RobotSpeed) {
  //   ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(RobotSpeed, 0.01);
  //   DifferentialDriveWheelSpeeds wheelSpeeds = WestCoastConstants.westCoastKinematic.toWheelSpeeds(targetSpeeds);
  //   leftModule.setModule(wheelSpeeds.leftMetersPerSecond/WestCoastConstants.maxDirveMotorSpeed);
  //   rightModule.setModule(wheelSpeeds.rightMetersPerSecond/WestCoastConstants.maxDirveMotorSpeed);
  // }

  public void resetGyro() {
    gyro.reset();
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  // public Pose2d getPose() {
  //   return odometry.getPoseMeters();
  // }

  // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(leftModule.getForwardVelocity(), rightModule.getForwardVelocity());//要MeterPerSecond
  // }

  // public DifferentialDriveWheelPositions getPosition() {
  //   return new DifferentialDriveWheelPositions(leftModule.getForwardPosition(), rightModule.getForwardPosition());
  // }

  // public ChassisSpeeds getSpeeds() {
  //   return WestCoastConstants.westCoastKinematic.toChassisSpeeds(getWheelSpeeds());
  // }


  // public void setPose(Pose2d pose) {
  //   odometry.resetPosition(getRotation2d(), getPosition(), pose);
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // odometry.update(getRotation2d(), rightModule.getForwardPosition(), leftModule.getForwardPosition());
  }
}
