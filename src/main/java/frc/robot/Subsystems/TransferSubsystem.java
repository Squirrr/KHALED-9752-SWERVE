// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class TransferSubsystem extends SubsystemBase {
  /** Creates a new TransferSubsystem. */
  final TalonFX transferMotor = new TalonFX(ShooterConstants.kTransferMotorPort, "rio");
  final Servo gateServo = new Servo(ShooterConstants.kServoPort);
  public TransferSubsystem() {}

  public void open() {
    gateServo.setPosition(1);
  }

  public void close() {
    gateServo.setPosition(0.6);
  }

  public void setTransfer(double speed) {
    transferMotor.set(speed);
  }

  public Command DefaultCommand() {
    return run(
      () -> {
        transferMotor.stopMotor();;
      });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
