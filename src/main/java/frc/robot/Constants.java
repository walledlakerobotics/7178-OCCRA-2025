// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MecanumDriveConstants {
    public static final Translation2d kFrontLeftTranslation2d = new Translation2d();
    public static final Translation2d kFrontRightTranslation2d = new Translation2d();
    public static final Translation2d kBackLeftTranslation2d = new Translation2d();
    public static final Translation2d kBackRightTranslation2d = new Translation2d();

    public static final int kFrontLeftSparkID = 2;
    public static final int kFrontRightSparkID = 4;
    public static final int kBackLeftSparkID = 8;
    public static final int kBackRightSparkID = 6;

    public static final double kForwardMaxSpeed = 6.5;
    public static final double kStrafeMaxSpeed = 2.5;
    
    public static final double kTurnMaxSpeed = 8;

    public static final double kMaxAcceleration = 7;

    public static final double kWheelBaseLength = 0.508;
    public static final double kTrackWidth = 0.5842;

    public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
      new Translation2d(kWheelBaseLength / 2, kTrackWidth / 2), 
      new Translation2d(kWheelBaseLength / 2, -kTrackWidth / 2), 
      new Translation2d(-kWheelBaseLength / 2, kTrackWidth / 2), 
      new Translation2d(-kWheelBaseLength / 2, -kTrackWidth / 2));


    public static final boolean kRightMotorsInverted = true;
    public static final boolean kLeftMotorsInverted = false;

    public static final double kPIDp = 0.1;
    public static final double kPIDi = 0.00;
    public static final double kPIDd = 0.00;

    public static final double kPosConvFactor = ((Units.inchesToMeters(6) * Math.PI) / 8.451);
    public static final double kVelConvFactor = kPosConvFactor / 60.0;
    
    public static final int kSmartCurrentLimit = 30;

    public static final double kXAxisSensitvity = 1;
    public static final double kYAxisSensitvity = 1;
    public static final double kRotationAxisSensitivity = 1;

    public static final double kDeadband = 0.01;
      
  }

  public static class ElevatorConstants {
    public static final int kElevatorSparkID = 62;
    public static final boolean kElevatorMotorInverted = true;

    public static final double kPIDp = 2.00;
    public static final double kPIDi = 0.00;
    public static final double kPIDd = 0.00;

    public static final double kPosConvFactor = 0.01;
    public static final double kVelConvFactor = kPosConvFactor/60.0;

    public static final double kMaxVelocity = 1.5;
    public static final double kMaxAcceleration = 1.5;

    public static final double kElevatorVelocityFactor = 1;
    public static final double kElevatorManualControlDeadband = 0.01;
  }

  public static class PneumaticsConstants {

    public static int kPCMID = 30;

    public static int kClawChannel = 6;

    public static int kFourBarChannel = 5;
  }

  public static class AutonConstants {
    public static final PIDConstants kPIDConstantsTranslation =  new PIDConstants(1,0,0);
    public static final PIDConstants kPIDConstantsRotation =  new PIDConstants(1,0,0);
    public static final PathFollowingController kPathFollowingController = new PPHolonomicDriveController(kPIDConstantsTranslation, kPIDConstantsRotation);
    public static final RobotConfig kRobotConfig;

    static {
      try {
        kRobotConfig = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        throw new RuntimeException(e);
      }
    }
  }

}
