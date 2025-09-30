// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MecanumDriveConstants;

import java.io.ObjectInputFilter.Config;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MecanumDriveSubsystem extends SubsystemBase {

  SparkMax m_FrontLeft = new SparkMax(MecanumDriveConstants.kFrontLeftSparkID, SparkLowLevel.MotorType.kBrushless);
  SparkMax m_FrontRight = new SparkMax(MecanumDriveConstants.kFrontRightSparkID, SparkLowLevel.MotorType.kBrushless);
  SparkMax m_BackLeft = new SparkMax(MecanumDriveConstants.kBackLeftSparkID, SparkLowLevel.MotorType.kBrushless);
  SparkMax m_BackRight = new SparkMax(MecanumDriveConstants.kBackRightSparkID, SparkLowLevel.MotorType.kBrushless);
  MecanumDrive m_Drive;
  
  Translation2d m_FrontLeftTranslation2d = null;
  Translation2d m_FrontRightTranslation2d = null;
  Translation2d m_BackLeftTranslation2d = null;
  Translation2d m_BackRightTranslation2d = null;

  RelativeEncoder m_FrontLeftEncoder;
  RelativeEncoder m_FrontRightEncoder;
  RelativeEncoder m_BackLeftEncoder;
  RelativeEncoder m_BackRightEncoder;

  SparkClosedLoopController m_FrontLeftClosedLoop;
  SparkClosedLoopController m_FrontRightClosedLoop;
  SparkClosedLoopController m_BackLeftClosedLoop;
  SparkClosedLoopController m_BackRightClosedLoop;


  MecanumDriveKinematics m_Kinematics = new MecanumDriveKinematics(
      m_FrontRightTranslation2d, m_FrontLeftTranslation2d, m_BackRightTranslation2d, m_BackLeftTranslation2d
      );

  public MecanumDriveSubsystem() {

    SparkMaxConfig config = new SparkMaxConfig();
      config
            .smartCurrentLimit(MecanumDriveConstants.kSmartCurrentLimit)
            .idleMode(null);

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
      m_Drive = new MecanumDrive(
          speed -> m_FrontLeftClosedLoop.setReference(speed, ControlType.kVelocity),
          speed -> m_BackLeftClosedLoop.setReference(speed, ControlType.kVelocity),
          speed -> m_FrontRightClosedLoop.setReference(speed, ControlType.kVelocity),
          speed -> m_BackRightClosedLoop.setReference(speed, ControlType.kVelocity));

      m_Drive.setMaxOutput(MecanumDriveConstants.kMaxMperS);
    
  }    

  public void drive(double xSpeed, double ySpeed, double turnSpeed){
    m_Drive.driveCartesian(xSpeed, ySpeed, turnSpeed);
  }

  public Command mecanumDrive(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, DoubleSupplier rotationSpeedSupplier) {
    return run(() -> {
      double xSpeed = xSpeedSupplier.getAsDouble();
      double ySpeed = ySpeedSupplier.getAsDouble();
      double zRotation = rotationSpeedSupplier.getAsDouble();

      drive(xSpeed, ySpeed, zRotation);

    });
  }
}
