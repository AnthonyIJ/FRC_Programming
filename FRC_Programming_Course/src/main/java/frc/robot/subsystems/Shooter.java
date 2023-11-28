// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.   

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.ShooterPreset;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;
  private RelativeEncoder flywheelEncoder;

  private CANSparkMax kickerMotor;

  private CANSparkMax pivotMotor;
  private AbsoluteEncoder pivotEncoder;

  private PIDController flywheelPID;
  private PIDController pivotPID;
  
  public Shooter() {
    topMotor = new CANSparkMax(ShooterConstants.kTopFlywheelMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(ShooterConstants.kBottomFlywheelMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.follow(topMotor, true);
    flywheelPID = new PIDController(0, 0, 0);
    flywheelEncoder = topMotor.getEncoder();
    flywheelEncoder.setVelocityConversionFactor(ShooterConstants.kFlywheelVelocityConversionFactor);
    

    kickerMotor = new CANSparkMax(ShooterConstants.kKickerMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    kickerMotor.setIdleMode(IdleMode.kCoast);

    pivotMotor = new CANSparkMax(ShooterConstants.kPivotMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(ShooterConstants.kPivotPositionConversionFactor);
    pivotPID = new PIDController(ShooterConstants.kPivotP, 0, 0);
  }

  public void setPivotSetpoint(double angleDegrees){
    pivotPID.setSetpoint(Units.degreesToRadians(angleDegrees));
  }

  public void setFlywheelSetpoint(double rpm){
    flywheelPID.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(rpm));
  }

  public double getPivotSetpoint(){
    return pivotPID.getSetpoint();
  }

  public double getFlywheelSetpoint(){
    return flywheelPID.getSetpoint();
  }

  public double getPivotAngleRadians(){
    return pivotEncoder.getPosition() * ShooterConstants.kPivotGearRatio;
  }

  public double getFlywheelVelocity(){
    return flywheelEncoder.getVelocity();
  }

  public void setKickerSpeed(double velocity){
    kickerMotor.set(velocity);
  }
  
  public Command intake(){
    return new ParallelCommandGroup(setPreset(ShooterConstants.kIntakePreset), new InstantCommand(() -> setKickerSpeed(ShooterConstants.kKickerIntakeMotorSpeed)));
  }

  public Command outtake(){
    return new SequentialCommandGroup(setPreset(ShooterConstants.kOuttakePreset), new InstantCommand(() -> setKickerSpeed(ShooterConstants.kKickerOuttakeMotorSpeed)));
  }

  public Command setPreset(ShooterPreset preset){
    return new InstantCommand(() -> {
      setFlywheelSetpoint(preset.FlywheelRPM);
      setPivotSetpoint(preset.PivotDegrees);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivotMotor.setVoltage(pivotPID.calculate(getPivotAngleRadians()));
    topMotor.setVoltage(flywheelPID.calculate(getFlywheelVelocity()));
  }
}
