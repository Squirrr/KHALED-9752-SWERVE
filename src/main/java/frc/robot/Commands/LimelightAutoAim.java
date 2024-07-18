// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.utils.ShooterPreset;
import frc.robot.utils.VisionLookUpTable;

public class LimelightAutoAim extends Command {
  LimelightSubsystem limelight;
  ArmSubsystem arm;

  private double limelightPos = 0;
  private VisionLookUpTable m_VisionLookUpTable;
  private ShooterPreset m_ShooterPreset;

  /** Creates a new LimelightAutoAim. */
  public LimelightAutoAim(LimelightSubsystem l, ArmSubsystem a) {
    limelight = l;
    arm = a;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(arm);
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
      System.out.println(limelight.detectedTargetDistance);
      m_ShooterPreset = m_VisionLookUpTable.getShooterPreset(limelight.detectedTargetDistance);
      double m_ArmPos = m_ShooterPreset.getArmAngle() * 56.1/90;
      // System.out.println("Arm Angle" + m_ShooterPreset.getArmAngle());
      // System.out.println("Arm Position:" + m_ArmPos);
      arm.setArmPosition(m_ArmPos*(45.0/25.0));
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
