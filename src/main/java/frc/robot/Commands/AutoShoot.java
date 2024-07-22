// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.GateSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.utils.ShooterPreset;
import frc.robot.utils.VisionLookUpTable;

public class AutoShoot extends Command {
  LimelightSubsystem limelight;
  ArmSubsystem arm;
  ShooterSubsystem shooter;
  GateSubsystem gate;
  TransferSubsystem transfer;

  private Timer transferTimer = new Timer();
  private VisionLookUpTable m_VisionLookUpTable;
  private ShooterPreset m_ShooterPreset;

  private boolean finished = false;
  private double m_leftShooter, m_rightShooter, m_armAngle, m_ArmPos;

  /** Creates a new LimelightAutoAim. */
  public AutoShoot(ArmSubsystem a, ShooterSubsystem s, 
  GateSubsystem g, TransferSubsystem t, LimelightSubsystem l) {
    limelight = l;
    arm = a;
    shooter = s;
    gate = g;
    transfer = t;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, arm, shooter, gate, transfer);
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
      // System.out.println(limelight.detectedTargetDistance);
      m_ShooterPreset = m_VisionLookUpTable.getShooterPreset(limelight.detectedTargetDistance);
      m_armAngle = m_ShooterPreset.getArmAngle();
      m_ArmPos =  m_armAngle * 56.1/90;
      m_leftShooter = m_ShooterPreset.getLeftShooter();
      m_rightShooter = m_ShooterPreset.getRightShooter();
      // System.out.println("Arm Angle" + m_ShooterPreset.getArmAngle());
      // System.out.println("Arm Position:" + m_ArmPos);
      arm.setArmPosition(m_ArmPos*(45.0/25.0));

      while (Math.round(arm.currentArmPos()*(25.0/45.0)) != Math.round(m_ArmPos)) {
          SmartDashboard.putNumber("Ll Angle", m_armAngle);
          SmartDashboard.putNumber("Distance", limelight.detectedTargetDistance);
        }
      
      transferTimer.restart();
      SmartDashboard.putNumber("transferTimer", transferTimer.get());
      while (transferTimer.get()<0.25) {
        SmartDashboard.putNumber("transferTimer", transferTimer.get());
        transfer.setTransfer(-0.1);
      }

      shooter.shooterSpeedReached = false;
      while (!shooter.shooterSpeedReached) {
      shooter.setShooterRPM(m_leftShooter, m_rightShooter);
      }
    gate.open();
    transfer.setTransfer(0.5);
    
    transferTimer.restart();
    while (transferTimer.get()<0.25) {
      shooter.setShooterRPM(m_leftShooter, m_rightShooter);
    }
    }
    finished = true;
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
