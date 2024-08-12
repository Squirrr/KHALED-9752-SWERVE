// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class TransferSubsystem extends SubsystemBase {
  /** Creates a new TransferSubsystem. */
  final TalonFX transferMotor = new TalonFX(ShooterConstants.kTransferMotorPort, "rio");
  private TalonFXConfiguration transferConfig = new TalonFXConfiguration();
  private TimeOfFlight firstSensor = new TimeOfFlight(0);


  public TransferSubsystem() {
    transferMotor.setNeutralMode(NeutralModeValue.Brake);
    firstSensor.setRangingMode(RangingMode.Short, 40);


    transferConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    transferConfig.CurrentLimits.SupplyCurrentLimit = 20;
    transferConfig.CurrentLimits.SupplyCurrentThreshold = 30;
    transferConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;
    transferMotor.getConfigurator().apply(transferConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double sensorDist() {
    return firstSensor.getRange();
  }
  public void setTransfer(double speed) {
    transferMotor.set(speed);
  }

  public boolean isIntaked(){
    if(firstSensor.getRange() < 200){
      return true;
    }
    return false;
  }

  public Command Transfer(double tSpeed) {
    return run(
      () -> {
        setTransfer(tSpeed);
      });
  }

  public Command DefaultCommand() {
    return run(
      () -> {
        transferMotor.stopMotor();;
      });
  }
}
