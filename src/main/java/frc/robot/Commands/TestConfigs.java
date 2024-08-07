// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.ArmSubsystem;
// import frc.robot.Subsystems.GateSubsystem;
// import frc.robot.Subsystems.LimelightSubsystem;
// import frc.robot.Subsystems.ShooterSubsystem;
// import frc.robot.Subsystems.TransferSubsystem;

// public class TestConfigs extends Command {

//   LimelightSubsystem limelight;
//   ArmSubsystem arm;
//   ShooterSubsystem shooter;
//   GateSubsystem gate;
//   TransferSubsystem transfer;


//   private double m_leftShooter, m_rightShooter;
//   /** Creates a new TestConfigs. */
//   public TestConfigs(LimelightSubsystem l, ArmSubsystem a, ShooterSubsystem s, 
//   GateSubsystem g, TransferSubsystem t) {
//     limelight = l;
//     arm = a;
//     shooter = s;
//     gate = g;
//     transfer = t;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(limelight, arm, shooter, gate, transfer);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double m_armAngle = 25.9;
//       double m_ArmPos =  m_armAngle * 56.1/90;
//       m_leftShooter = 4000;
//       m_rightShooter = 6000;
//       // System.out.println("Arm Angle" + m_ShooterPreset.getArmAngle());
//       // System.out.println("Arm Position:" + m_ArmPos);
//       arm.setArmPosition(m_ArmPos*(45.0/25.0));

//       while (Math.round(arm.currentArmPos()*(25.0/45.0)) != Math.round(m_ArmPos)) {
//           SmartDashboard.putNumber("Test Angle", m_armAngle);
//           SmartDashboard.putNumber("Distance", limelight.detectedTargetDistance);
//         }

//       shooter.shooterSpeedReached = false;
//       while (!shooter.shooterSpeedReached) {
//       shooter.setShooterRPM(m_leftShooter, m_rightShooter);
//       }
//     gate.open();
//     transfer.setTransfer(0.5);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
