// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.MecanumDriveConstants;

public class MecanumDriveSubsystem extends SubsystemBase {

  //Drive Motors
  final SparkMax m_FrontLeft = new SparkMax(
      MecanumDriveConstants.kFrontLeftSparkID,
      SparkLowLevel.MotorType.kBrushless
      );
  final SparkMax m_FrontRight = new SparkMax(
      MecanumDriveConstants.kFrontRightSparkID,
      SparkLowLevel.MotorType.kBrushless
      );
  final SparkMax m_BackLeft = new SparkMax(
      MecanumDriveConstants.kBackLeftSparkID, 
      SparkLowLevel.MotorType.kBrushless
      );
  final SparkMax m_BackRight = new SparkMax(
    MecanumDriveConstants.kBackRightSparkID,
    SparkLowLevel.MotorType.kBrushless
    );

  //Gyro For PathPlanner
  final AHRS m_Gyro = new AHRS(NavXComType.kMXP_SPI);

  //Translations from bot center of wheels
  final Translation2d m_FrontLeftTranslation2d = new Translation2d(
    MecanumDriveConstants.kWheelBaseLength / 2, 
    MecanumDriveConstants.kTrackWidth / 2
    );
  final Translation2d m_FrontRightTranslation2d = new Translation2d(
    MecanumDriveConstants.kWheelBaseLength / 2,
    -MecanumDriveConstants.kTrackWidth / 2
    );
  final Translation2d m_BackLeftTranslation2d = new Translation2d(
    -MecanumDriveConstants.kWheelBaseLength / 2, 
    MecanumDriveConstants.kTrackWidth / 2
    );
  final Translation2d m_BackRightTranslation2d = new Translation2d(
    -MecanumDriveConstants.kWheelBaseLength / 2, 
    -MecanumDriveConstants.kTrackWidth / 2
    );

  //Encoders
  final RelativeEncoder m_FrontLeftEncoder;
  final RelativeEncoder m_FrontRightEncoder;
  final RelativeEncoder m_BackLeftEncoder;
  final RelativeEncoder m_BackRightEncoder;

  //Closedloop Controllers of Drive Motors
  final SparkClosedLoopController m_FrontLeftClosedLoop;
  final SparkClosedLoopController m_FrontRightClosedLoop;
  final SparkClosedLoopController m_BackLeftClosedLoop;
  final SparkClosedLoopController m_BackRightClosedLoop;

  //Odometry Object
  final MecanumDriveOdometry m_Odometry;

  //Kinematics Object
  MecanumDriveKinematics m_Kinematics = new MecanumDriveKinematics(
      m_FrontRightTranslation2d, 
      m_FrontLeftTranslation2d, 
      m_BackRightTranslation2d, 
      m_BackLeftTranslation2d
      );


  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
        m_FrontLeftEncoder.getPosition(),
        m_FrontRightEncoder.getPosition(),
        m_BackLeftEncoder.getPosition(),
        m_BackRightEncoder.getPosition());
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_FrontLeftEncoder.getVelocity(),
        m_FrontRightEncoder.getVelocity(),
        m_BackLeftEncoder.getVelocity(),
        m_BackRightEncoder.getVelocity());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_Kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public MecanumDriveSubsystem() {
    
    SparkMaxConfig config = new SparkMaxConfig();
      config
            .smartCurrentLimit(MecanumDriveConstants.kSmartCurrentLimit)
            .idleMode(IdleMode.kBrake);
      config.closedLoop
            .pid(MecanumDriveConstants.kPIDp, MecanumDriveConstants.kPIDi, MecanumDriveConstants.kPIDd)
            .velocityFF(1/MecanumDriveConstants.kKVConstant);
      config.encoder
            .positionConversionFactor(MecanumDriveConstants.kPosConvFactor)
            .velocityConversionFactor(MecanumDriveConstants.kVelConvFactor);

      config.inverted(MecanumDriveConstants.kLeftMotorsInverted);

      m_BackLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_FrontLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      config.inverted(MecanumDriveConstants.kRightMotorsInverted);

      m_BackRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_FrontRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      //get encoders
      m_BackLeftEncoder = m_BackLeft.getEncoder();
      m_BackRightEncoder = m_BackRight.getEncoder();
      m_FrontLeftEncoder = m_FrontLeft.getEncoder();
      m_FrontRightEncoder = m_FrontRight.getEncoder();
      //get closed loop controllers
      m_BackLeftClosedLoop = m_BackLeft.getClosedLoopController();
      m_BackRightClosedLoop = m_BackRight.getClosedLoopController();
      m_FrontLeftClosedLoop = m_FrontLeft.getClosedLoopController();
      m_FrontRightClosedLoop = m_FrontRight.getClosedLoopController();
      //drive system

      m_Odometry = new MecanumDriveOdometry(m_Kinematics, m_Gyro.getRotation2d(), getWheelPositions());
      
      AutoBuilder.configure(
          m_Odometry::getPoseMeters, 
          this::resetOdometry, 
          this::getChassisSpeeds, 
          this::driveRobotRelative, 
          AutonConstants.kPathFollowingController, 
          AutonConstants.kRobotConfig,
          () -> false,
          this);
  }

  public void driveRobotRelative(ChassisSpeeds Speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = m_Kinematics.toWheelSpeeds(Speeds);
    m_FrontLeftClosedLoop.setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
    m_BackLeftClosedLoop.setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
    m_FrontRightClosedLoop.setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
    m_BackRightClosedLoop.setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
  }

  public void drive(double xSpeed, double ySpeed, double turnSpeed) {
    ChassisSpeeds C = new ChassisSpeeds(
        xSpeed * MecanumDriveConstants.kForwardMaxSpeed, 
        ySpeed * MecanumDriveConstants.kStrafeMaxSpeed, 
        turnSpeed * MecanumDriveConstants.kTurnMaxSpeed
    );
    driveRobotRelative(C);
  }

  public Command mecanumDrive(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotationSpeedSupplier) {
    return run(() -> {
       double xSpeed = xSpeedSupplier.getAsDouble();
       double ySpeed = ySpeedSupplier.getAsDouble(); 
       double zRotation = rotationSpeedSupplier.getAsDouble();

       drive(xSpeed, ySpeed, -zRotation);

    });
  }

  @Override
  public void periodic() {
    m_Odometry.update(m_Gyro.getRotation2d(), getWheelPositions());
  }

  public void resetOdometry(Pose2d pose) {
    m_Odometry.resetPosition(m_Gyro.getRotation2d(), getWheelPositions(), pose);
  }
}
