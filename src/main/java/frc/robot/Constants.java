package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
// import frc.robot.utils.ShooterConfig;
// import frc.robot.utils.ShooterPreset;
import frc.robot.utils.ShooterConfig;
import frc.robot.utils.ShooterPreset;

//FRC 9752

public final class Constants {
    /* As per old code, given by Coach Abdurrehman vvv*/
    public static final double stickDeadband = 0.1;
    public static final int CANdleID = 1;
    public static final int JoystickId = 0;
    public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
    public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
    public static final int BlockButton = XboxController.Button.kStart.value;
    public static final int MaxBrightnessAngle = 90;
    public static final int MidBrightnessAngle = 180;
    public static final int ZeroBrightnessAngle = 270;
    public static final int VbatButton = XboxController.Button.kA.value;
    public static final int V5Button = XboxController.Button.kB.value;
    public static final int CurrentButton = XboxController.Button.kX.value;
    public static final int TemperatureButton = XboxController.Button.kY.value;
    public static final boolean kIsTuningMode = true;
    
    public static final double headingPresetKp = 0.003*3.0;
    public static final double headingPresetKd = 0.0001*3.0;
    public static final double driveHeadingKp = 0.003*3.0;
    public static final double driveHeadingKd = 0.0001*3.0;

    public static final double translationalAutoP = 3.8;
    public static final double rotationalAutoP = 6.25;

    public static final class Universal {
        public static final double voltageMin = 0;
        public static final double voltageMax = 12;
    }

    // public static ShooterConfig getLookupTable(){
    //     ShooterConfig shooterConfig = new ShooterConfig();
    //     shooterConfig.getShooterConfigs().add(new ShooterPreset(6.5, 2500, 2500, 1.9)); // Distance -> Bumper
    //     shooterConfig.getShooterConfigs().add(new ShooterPreset(10, 2500, 3500, 19.54)); // Distance -> Bumper
    //     shooterConfig.getShooterConfigs().add(new ShooterPreset(11.5, 2500, 4000, 36.27)); // Distance -> Bumper
    //     shooterConfig.getShooterConfigs().add(new ShooterPreset(12.6, 2500, 4500, 54.75)); // Distance -> Bumper
    //     shooterConfig.getShooterConfigs().add(new ShooterPreset(13.5, 2500, 4500, 75)); // Distance -> Bumper
    //     shooterConfig.getShooterConfigs().add(new ShooterPreset(14.5, 2500, 5500, 92)); // Distance -> Bumper
    //     shooterConfig.getShooterConfigs().add(new ShooterPreset(15.2, 3500, 5500, 110)); // Distance -> Bumper
    //     shooterConfig.getShooterConfigs().add(new ShooterPreset(15.75, 3500, 6000, 128.4)); // Distance -> Bumper
    //     return shooterConfig;
    // }

    public static ShooterConfig getLookupTable(){
        ShooterConfig shooterConfig = new ShooterConfig();
        shooterConfig.getShooterConfigs().add(new ShooterPreset(10, 2500, 2500, 1.58)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(15.5, 2500, 3500, 20.50)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(18, 2500, 4000, 36.71)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(21, 2500, 4500, 54.84)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(22.8, 3000, 4500, 75.04)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(23, 3000, 5500, 92.12)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(24.5, 3500, 5500, 110.18)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(25.9, 4000, 6000, 127.54)); // Distance -> Bumper
        return shooterConfig;
    }

    public static final class LimelightConstants {
        public static final String photonVisionName = "9752limelight";
        public static final double limelightMountAngleDegrees = 22.3;
        public static final double limelightLensHeightInches = 19;
        public static final double goalHeightInches = 57;
        public static final double cameraToSpeakerDistance = 44;
        public static final double turnPID = 0.005;
        public static final double heightToAprilTag = 38.625; //Dist from Ll to AprilTag
        public static final double aprilTagToSpeakerHeight = 20;
    }
    /* As per old code, given by Coach Abdurrehman ^^^*/

    public static double degreesToArmAngle(double degrees, double gearRatio) {
        return degrees*56.1/90; //Returns rotations
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorPort = 15;
    }

    public static final class ArmConstants {
        public static final int kArmMotorPort = 16;
        public static final double ampPos = 70;
        public static final double subPos = 10;

        public class ArmPIDConstants {
            //ArmPID came from Rayyan b/c his worked more smoothly than mine
            public static final double kG = 0.255;
            public static final double kP = 1.5;
            public static final double kI = 0;
            public static final double kD = 0.2;
        }
    }

    public static final class ShooterConstants {
        public static final int kTransferMotorPort = 20; 
        public static final int kLeftShooterMotorPort = 21; //SparkMAX Controller 
        public static final int kRightShooterMotorPort = 20; //SparkMAX Controller
        public static final int kServoPort = 9;
        
        public class ShooterPIDConstants {
            public static final double kFF = 0.002;
            public static final double kP = 0.002;
            public static final double kI = 0.0;
            public static final double kD = 0.001;
        }
    }
    /*Swerve constants taken form 9752worlds*/
    public static final class Swerve {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19.2); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelBase = Units.inchesToMeters(19.2); // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 20;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 50;
        public static final int driveCurrentThreshold = 80;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.2; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 6; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 14; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final double absolutePosition = -0.0283203125;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, absolutePosition);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final double absolutePosition = -0.0263671875;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, absolutePosition);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final double absolutePosition = -0.03662109375+0.5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, absolutePosition);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final double absolutePosition = 0.278564453125+0.5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, absolutePosition);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 5.9436;
        public static final double kMaxAccelerationMetersPerSecondSquared = 14;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 2;

        public static final double kPXController = 3;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 2.5;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class LoggerSubscription {
        public static final String battery = "BATTERY";
        public static final String error = "ERROR";
        public static final String motor = "MOTOR";
        public static boolean batteryIsSubbed = false;
        public static boolean errorIsSubbed = false;
        public static boolean motorIsSubbed = false;
        public static final double lowBatteryVoltage = 11.5;
        public static final double lowMotorVoltage = 12;
    }
}