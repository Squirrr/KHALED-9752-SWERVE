// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
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
    public static boolean limelightTargeted, idle;
    StrobeAnimation strobeIntake = new StrobeAnimation(0,255,0,0,.7,999);
    
    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.5;
        candle.configAllSettings(config);
    }

    @Override
    public void periodic() {
        /* Variables so it's easier to understand code */
        currentArmPos = RobotContainer.arm.currentArmPos();
        sensorDist = RobotContainer.transfer.sensorDist();
        limelightTargeted = LimelightHelpers.getTV("limelight");
        targetedDist = RobotContainer.limelight.detectedTargetDistance;
        intakeState = (RobotContainer.intake.intakeSpeed()>=0.05) ? 1 
        : (RobotContainer.intake.intakeSpeed()<=-0.05) ? -1 : 0; //1 = intaking, -1 = outtaking, 0 = N/A
        
        if (Robot.disabled) { //When disabled, change color to blue if blue alliance, red if red alliance
            switch (getAllianceAsNum()) {
                case 2: //RED ALLIANCE
                candle.clearAnimation(0);
                candle.setLEDs(255, 0, 0); //RED
                break;

                case 1: //BLUE ALLIANCE
                candle.clearAnimation(0);
                candle.setLEDs(0, 0, 255); //BLUE
                break;
                
                default: //NO ALLIANCE?
                candle.clearAnimation(0);
                candle.setLEDs(255, 0, 255); //PURPLE
                break;
            }
        } else {

            if (Math.round(currentArmPos) == ArmConstants.ampPos) { //AMP
                idle = false;
                candle.clearAnimation(0);
                candle.setLEDs(255, 125, 0); //ORANGE

            } else if (Math.round(currentArmPos) == ArmConstants.subPos) { //CLOSESHOT
                idle = false;
                candle.clearAnimation(0);
                candle.setLEDs(50, 100, 50); //BLUE

            } else if(Math.round(currentArmPos/3) == 0) { //INTAKE POSITION
                if (limelightTargeted && targetedDist < 100) {  //If within range of limelight
                    if (intakeState == 1 /*&& !(sensorDist < 250)*/) { //If intaking and note is NOT in shooter
                        idle = false;
                        candle.clearAnimation(0);
                        candle.setLEDs(0, 255, 0); //GREEN

                    } else { //If within range but not intaking or note is in shooter 
                        idle = false;
                        candle.clearAnimation(0);
                        candle.setLEDs(255, 255, 255); //WHITE
                    }
                } /*else if (sensorDist < 250) { //If not in range but note acquuired
                    candle.clearAnimation(0);
                    candle.setLEDs(255, 0, 255); //PURPLE

                }*/ else if (intakeState == 1) { //If not in range but intaking
                    idle = false;
                    candle.clearAnimation(0);
                    candle.setLEDs(50, 255, 50); //GREEN

                } else if (intakeState == -1) { //If not in range but outtaking
                    idle = false;
                    candle.clearAnimation(0);
                    candle.setLEDs(255, 50, 50); //RED
                    
                } else {
                    SingleFadeAnimation singleFadeAnimation = new SingleFadeAnimation(0, 255, 200, 0, 0.5, 999);
                    // LarsonAnimation larsonAnimation = new LarsonAnimation(255, 0, 255, 0, 0.5, 999, BounceMode.Front, 7);
                    idle = true;
                    if (!idle) {candle.clearAnimation(0);}
                    candle.animate(singleFadeAnimation);
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
                //Test command
              });
  }

    public Command defaultCommand(){
      return run(
              () -> {
                //I think this will allow the LEDs to run passively since all the code's in periodic.
              });
  }
}
