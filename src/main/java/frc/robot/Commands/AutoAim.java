// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class AutoAim extends Command {
  LimelightSubsystem limelight;
  ArmSubsystem arm;
  double limelightPos;
  /** Creates a new AutoAim. */
  public AutoAim(LimelightSubsystem l, ArmSubsystem a) {
    limelight = l;
    arm = a;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelightPos = limelight.setLimelightArmPos();
    System.out.println(limelightPos);
    SmartDashboard.putNumber("Limelight Pos", limelightPos);
    arm.setArmPosition(limelightPos);
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
