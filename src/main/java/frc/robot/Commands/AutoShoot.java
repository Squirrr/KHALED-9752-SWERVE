// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.GateSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.utils.ShooterPreset;
import frc.robot.utils.VisionLookUpTable;

public class AutoShoot extends Command {
  /** Creates a new AutoShoot. */
  LimelightSubsystem limelight;
  ShooterSubsystem shooter;
  TransferSubsystem transfer;
  GateSubsystem gate;

  private VisionLookUpTable m_VisionLookUpTable;
  private ShooterPreset m_ShooterPreset;

  private boolean finished = false;
  private double m_leftShooter, m_rightShooter;

  public AutoShoot(ShooterSubsystem s, LimelightSubsystem l) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_VisionLookUpTable = new VisionLookUpTable();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight")) {
      m_ShooterPreset = m_VisionLookUpTable.getShooterPreset(limelight.detectedTargetDistance);

      shooter.shooterSpeedReached = false;
      shooter.setShooterRPM(m_leftShooter, m_rightShooter);

      while (!shooter.shooterSpeedReached) {
          SmartDashboard.putNumber("L Shooter Speed", shooter.currentShooterSpeed()[0]);
          SmartDashboard.putNumber("R Shooter Speed", shooter.currentShooterSpeed()[1]);
      }

      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
