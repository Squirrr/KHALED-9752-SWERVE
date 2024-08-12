// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public static final TalonFX armMotor = new TalonFX(ArmConstants.kArmMotorPort, "static");

  TalonFXConfiguration armConfigs = new TalonFXConfiguration();
  Slot0Configs slot0Configs = armConfigs.Slot0;
  MotionMagicConfigs motionMagicConfigs = armConfigs.MotionMagic;
  MotionMagicVoltage request;

  public boolean armPositionReached = false;

  public ArmSubsystem() {
    armConfigs.CurrentLimits.SupplyCurrentThreshold = 20;
        armConfigs.CurrentLimits.SupplyCurrentLimit = 10;
        armConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    armMotor.getConfigurator().apply(armConfigs);
    armMotor.setNeutralMode(NeutralModeValue.Brake);

    var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.3; // Add 0.25 V output to overcome static friction
        //9752 needs this
        slot0Configs.kP = 3.0; // A position error of 2.5 rotations results in 12 V output

        //9128 robot needs this:
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
  
      armMotor.getConfigurator().apply(talonFXConfigs);
      
      request = new MotionMagicVoltage(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmPosition(double positionSetPoint) {
    armPositionReached = false;
    armMotor.setControl(request.withPosition(positionSetPoint).withSlot(0));
    if (Math.round(armMotor.getPosition().getValueAsDouble())==positionSetPoint) {
      armPositionReached = true;
    }
  }

  public boolean armPosReached() {
    return armPositionReached;
  }

  public Command SetArmToPos(double pos) {
    return run(
      () -> {
        setArmPosition(pos);
      });
  }

  public double currentArmPos() {
    return armMotor.getPosition().getValueAsDouble();
  }
}
