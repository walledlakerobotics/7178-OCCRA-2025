// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MecanumDriveConstants;

public class ElevatorSubsystem extends SubsystemBase {

  SparkMax m_ElevatorSparkMax = new SparkMax(ElevatorConstants.kElevatorSparkID, MotorType.kBrushless);
  SparkClosedLoopController m_ElevatorClosedLoop;
  RelativeEncoder m_ElevatorEncoder;

  public ElevatorSubsystem() {
    m_ElevatorClosedLoop = m_ElevatorSparkMax.getClosedLoopController();
    m_ElevatorEncoder = m_ElevatorSparkMax.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
      config
            .smartCurrentLimit(MecanumDriveConstants.kSmartCurrentLimit)
            .idleMode(IdleMode.kBrake);

      config.closedLoop
            .pid(ElevatorConstants.kPIDp, ElevatorConstants.kPIDi, ElevatorConstants.kPIDd)
            .velocityFF(1/MecanumDriveConstants.kKVConstant);

      config.inverted(ElevatorConstants.kElevatorMotorInverted);

  }

  public Command elevatorSetPosition(double position) {
    return runOnce(
        () -> {
            m_ElevatorClosedLoop.setReference(position, ControlType.kPosition);
        });
  }

  public Command elevatorManualPosition(DoubleSupplier velocitySupplier) {
    return run(
        () -> {
            double velocity = MathUtil.applyDeadband(
                velocitySupplier.getAsDouble() , ElevatorConstants.kElevatorManualControlDeadband) * 
                ElevatorConstants.kElevatorVelocityFactor;
            m_ElevatorClosedLoop.setReference(velocity, ControlType.kVelocity);
        });
  }

}


