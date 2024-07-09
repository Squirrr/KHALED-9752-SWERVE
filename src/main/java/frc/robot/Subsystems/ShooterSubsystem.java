// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  final CANSparkFlex rightShooterMotor = new CANSparkFlex(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);
  final CANSparkFlex leftShooterMotor = new CANSparkFlex(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);

  final RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();
  final RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();

  final SparkPIDController rightPIDController = rightShooterMotor.getPIDController();
  final SparkPIDController leftPIDController = leftShooterMotor.getPIDController();

  public boolean shooterSpeedReached = false;

  public ShooterSubsystem() {
    rightShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.restoreFactoryDefaults();

    rightPIDController.setFF(ShooterPIDConstants.kFF);
    rightPIDController.setP(ShooterPIDConstants.kP);
    rightPIDController.setI(ShooterPIDConstants.kI);
    rightPIDController.setD(ShooterPIDConstants.kD);

    leftPIDController.setFF(ShooterPIDConstants.kFF);
    leftPIDController.setP(ShooterPIDConstants.kP);
    leftPIDController.setI(ShooterPIDConstants.kI);
    leftPIDController.setD(ShooterPIDConstants.kD);

    rightShooterMotor.burnFlash();
    leftShooterMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command DefaultCommand() {
    return run(
      () -> {
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
      });
  }

  public void setShooterRPM(double leftFlywheelSetPoint, double rightFlywheelSetPoint) {
    leftPIDController.setReference(leftFlywheelSetPoint, ControlType.kVelocity);
    rightPIDController.setReference(-rightFlywheelSetPoint, ControlType.kVelocity);
    if (leftShooterEncoder.getVelocity() > leftFlywheelSetPoint*0.9 &&
        leftShooterEncoder.getVelocity() < leftFlywheelSetPoint*1.1) {
      shooterSpeedReached = true;
    } else {
      shooterSpeedReached = false;
    }
  }
  public double[] currentShooterSpeed() {
    double[] shooterVelocities = 
      {leftShooterEncoder.getVelocity(), rightShooterEncoder.getVelocity()};
    return shooterVelocities;
  }
}
