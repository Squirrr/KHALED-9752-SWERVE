// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LimelightConstants;

/** Add your docs here. */
public class LEDSubsystem extends SubsystemBase {
    public final static CANdle candle = new CANdle(Constants.CANdleID, "static");
    public static double currentArmPos, sensorDist, targetedDist, intakeState;
    public static boolean limelightTargeted;
    StrobeAnimation strobeIntake = new StrobeAnimation(0,255,0,0,.7,999);
    
    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.5;
        candle.configAllSettings(config);
    }

    @Override
    public void periodic() {
        currentArmPos = RobotContainer.arm.currentArmPos();
        sensorDist = RobotContainer.transfer.sensorDist();
        limelightTargeted = LimelightHelpers.getTV(LimelightConstants.photonVisionName);
        targetedDist = RobotContainer.limelight.detectedTargetDistance;
        intakeState = (RobotContainer.intake.intakeSpeed()>=0.05) ? 1 
        : (RobotContainer.intake.intakeSpeed()<=-0.05) ? -1 : 0; //1 = intaking, -1 = outtaking, 0 = N/A

        if (Robot.disabled) {
            switch (getAllianceAsNum()) {
                case 2: //RED
                candle.clearAnimation(0);
                candle.setLEDs(255, 0, 0);
                break;

                case 1: //BLUE
                candle.clearAnimation(0);
                candle.setLEDs(0, 0, 255);
                break;
                
                default: //NONE
                candle.clearAnimation(0);
                candle.setLEDs(255, 255, 0);
                break;
            }
        } else {

            if (Math.round(currentArmPos) == ArmConstants.ampPos) { //AMP = ORANGE
                candle.clearAnimation(0);
                candle.setLEDs(255, 50, 0);

            } else if (Math.round(currentArmPos) == ArmConstants.subPos) { //CLOSESHOT = BLUE
                candle.clearAnimation(0);
                candle.setLEDs(0, 0, 255);

            } else if(Math.round(currentArmPos/3) == 0){

                if (limelightTargeted && targetedDist < 100){
                    if (intakeState == 1 && !(sensorDist < 250)) {
                        candle.clearAnimation(0);
                        candle.setLEDs(0, 255, 0);

                    } else {
                        candle.clearAnimation(0);
                        candle.setLEDs(255, 255, 255);
                    }
                } else if (sensorDist < 250) {
                    candle.clearAnimation(0);
                    candle.setLEDs(255, 0, 255);

                } else if (intakeState == 1) {
                    candle.clearAnimation(0);
                    candle.setLEDs(0, 255, 0);

                } else if (intakeState == -1) {
                    candle.clearAnimation(0);
                    candle.setLEDs(255, 0, 0);
                    
                } else {
                    TwinkleAnimation twinkleAnimation = new TwinkleAnimation(50, 50, 50, 0, 0.5, 300,TwinklePercent.Percent42);
                    candle.animate(twinkleAnimation);
                }
            }
        }
    }

    public int getAllianceAsNum() {
        var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        if (alliance.get() == DriverStation.Alliance.Red) {
                            return 2;
                        } else if (alliance.get() == DriverStation.Alliance.Blue) {
                            return 1;
                        }
                    }
                    return 0;
    }

    public Command setLEDs(int r, int g, int b){
      return run(
              () -> {
                candle.clearAnimation(0);
                candle.setLEDs(r, g, b);
                //I think this will allow the LEDs to run passively since all the code's in periodic.
              });
  }

    public Command defaultCommand(){
      return run(
              () -> {
                //I think this will allow the LEDs to run passively since all the code's in periodic.
              });
  }
}
