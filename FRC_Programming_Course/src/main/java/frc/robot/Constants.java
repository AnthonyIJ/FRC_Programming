// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.utils.ShooterPreset;

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
  public static final class FlywheelConstants {
    public static double kSparkMaxP = 0.01;
    public static double kSparkMaxFeedforward = 0.01; // .00022

    public static int kLeftMotorPort = 11;
    public static int kRightMotorPort = 12;

    public static double kVelocityTolerance = 115;
  }

  public static final class ShooterConstants {
    //Kicker
    public static final int kKickerMotorCanId = 13;
    public static final int kKickerMotorCurrentLimit = 30;
    public static final double kKickerNominalVoltage = 12.0;
    public static final double kKickerIntakeMotorSpeed = 0.3;
    public static final double kKickerOuttakeMotorSpeed = -0.6;
    public static final double kKickerHoldMotorSpeed = 0.1;
    public static final double kKickSpeed = -0.4;

    //Pivot
    public static final int kPivotMotorCanId = 15;
    public static final double kPivotGearRatio = 50;
    public static final double kPivotPositionConversionFactor = (2*Math.PI) * kPivotGearRatio;
    public static final double kPivotEncoderZeroOffset = 220.0;
    public static final double kPivotKinematicOffset = 115;
    public static final int kPivotMotorCurrentLimit = 30;
    public static final double kPivotP = 3.0;
    public static final ArmFeedforward kPivotFeedforward = new ArmFeedforward(0, 0.49, 0.97, 0.01);
    
    //Flywheels
    public static final int kTopFlywheelMotorCanId = 16;
    public static final int kBottomFlywheelMotorCanId = 17;
    public static final double kFlywheelVelocityConversionFactor = (2*Math.PI) * 1.0;
    public static final int kTopFlywheelMotorCurrentLimit = 30;
    public static final int kBottomFlywheelMotorCurrentLimit = 30;

    //Preset Angles
    public static final double kPivotHoldAngleDegrees = -40;

    //Shooter Presets
    public static final ShooterPreset kIntakePreset = 
      new ShooterPreset(115, -100);
    public static final ShooterPreset kOuttakePreset = 
      new ShooterPreset(90, 50);

    public static final ShooterPreset kRetractPreset = 
      new ShooterPreset(-95, 0);
    public static final ShooterPreset kHoldPreset = 
      new ShooterPreset(kPivotHoldAngleDegrees, 0);

    public static final ShooterPreset kLaunchCubePreset = 
      new ShooterPreset(45, 170);

    public static final ShooterPreset kCloseHighCubePreset = 
      new ShooterPreset(15, 70);
    public static final ShooterPreset kFarHighCubePreset = 
      new ShooterPreset(45, 0);

    public static final ShooterPreset kCloseMiddleCubePreset = 
      new ShooterPreset(20, 40);
    public static final ShooterPreset kFarMiddleCubePreset = 
      new ShooterPreset(45, 0);
  }
}
