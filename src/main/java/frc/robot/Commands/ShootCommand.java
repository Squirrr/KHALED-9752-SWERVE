// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.GateSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TransferSubsystem;

public class ShootCommand extends Command {
  TransferSubsystem transfer;
  ShooterSubsystem shooter;
  GateSubsystem gate;
  double flywheelSpeed;
  /** Creates a new SimpleShoot. */
  public ShootCommand(TransferSubsystem t, ShooterSubsystem s, GateSubsystem g, double fSpeed) {
    transfer = t;
    shooter = s;
    gate = g;
    flywheelSpeed = fSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(transfer);
    addRequirements(shooter);
    addRequirements(gate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterRPM(flywheelSpeed*0.9, flywheelSpeed);
    if (shooter.shooterSpeedReached) {
      gate.open();
      transfer.setTransfer(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
