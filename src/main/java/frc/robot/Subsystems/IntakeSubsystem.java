// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorPort, "static");
  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  
  public IntakeSubsystem() {
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 80;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 80;
    intakeConfig.CurrentLimits.SupplyCurrentThreshold = 82;
    intakeConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime; 
    intakeMotor.getConfigurator().apply(intakeConfig);

    intakeMotor.setNeutralMode(NeutralModeValue.Coast);

  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public double intakeSpeed() {
    return intakeMotor.get();
  }

  public Command DefaultCommand() {
    return run(
      () -> {
        intakeMotor.set(0);
      });
  }

  public Command Intake() {
    return run(
      () -> {
        setIntake(1);
      });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
