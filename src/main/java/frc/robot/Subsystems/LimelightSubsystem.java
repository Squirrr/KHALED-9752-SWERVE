// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */

  public double limelightArmPos;
  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Limelight Arm Pos", limelightArmPos);
    // System.out.println(limelightArmPos);
    // limelightArmPos = setLimelightArmPos();


  }

  public double setLimelightArmPos() {
    double distance = (38.625/Math.tan(
      Math.toRadians(LimelightHelpers.getTY("limelight"))+10))+10;
    double llAngle = Math.toDegrees(Math.atan((38.625+20)/distance));
    double angle = 70-llAngle;
    double posRatio = 56.1/90;
    double testPos = angle * posRatio;
    return testPos;
  }

  public double limelight_aim_proportional() {    

    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .007; //TODO: Tune for specific robot

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    // targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  public Command DefaultCommand() {
    return run(
      () -> {
      });
  }
}