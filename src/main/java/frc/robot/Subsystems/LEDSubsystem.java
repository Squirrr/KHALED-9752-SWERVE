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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class LEDSubsystem extends SubsystemBase {
    public final static CANdle candle = new CANdle(Constants.CANdleID, "static");
    public static double currentArmPos = 0;
    StrobeAnimation strobeIntake = new StrobeAnimation(0,255,0,0,.7,999);

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1.0;
        candle.configAllSettings(config);
    }

    @Override
    public void periodic() {
        currentArmPos = RobotContainer.arm.currentArmPos();
        
        if (Math.round(RobotContainer.intake.intakeMotor.get())>0.05) {
            candle.clearAnimation(0);
            StrobeAnimation strobeAnimation = new StrobeAnimation(0,255,0,0,.5,999);
            candle.animate(strobeAnimation);
        } else if (Math.round(currentArmPos*5)!=0) {
            switch (Math.toIntExact(Math.round(currentArmPos*10))) {
                case 700: //AMP
                    candle.clearAnimation(0);
                    candle.setLEDs(255, 50, 0);
                    break;
                case 100: //SUB
                    candle.clearAnimation(0);
                    candle.setLEDs(255, 0, 255);
                    break;
                
                default: //LIMELIGHT
                    candle.clearAnimation(0);
                    candle.setLEDs(255, 255, 0);
                    break;
            }
        } else {
            candle.clearAnimation(0);
            TwinkleAnimation twinkleAnimation = new TwinkleAnimation(50, 50, 50, 0, 0.5, 208,TwinklePercent.Percent42);
            candle.animate(twinkleAnimation);
        }
    }

    public Command defaultCommand(){
      return run(
              () -> {
                //Skibidi sigma fanumtax no diddy fr
              });
  }
}
