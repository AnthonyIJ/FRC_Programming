// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel extends SubsystemBase {
  CANSparkMax motor1;
  CANSparkMax motor2;
  AbsoluteEncoder encoder;
  PIDController pid;
  SimpleMotorFeedforward ff;
  double targetSpeed;

  /** Creates a new Flywheel. */
  public Flywheel() {
    motor1 = new CANSparkMax(FlywheelConstants.kLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor2 = new CANSparkMax(FlywheelConstants.kRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor2.follow(motor1, true);
    encoder = motor1.getAbsoluteEncoder(Type.kDutyCycle);
    pid = new PIDController(FlywheelConstants.kSparkMaxP, 0, 0); //tune
    ff = new SimpleMotorFeedforward(0, FlywheelConstants.kSparkMaxFeedforward);
    targetSpeed = 0;
  }

  public void setMotorSpeed(double v){
    pid.setSetpoint(v);
    targetSpeed = v;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Goal: " , pid.getSetpoint());
    motor1.setVoltage(pid.calculate(encoder.getVelocity()) + ff.calculate(targetSpeed));
  }
}
