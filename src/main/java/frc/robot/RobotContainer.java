package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Autos.exampleAuto;
import frc.robot.Commands.AutoAim;
import frc.robot.Commands.SimpleShoot;
import frc.robot.Commands.SmartIntake;
import frc.robot.Commands.SmartOuttake;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.utils.SquaredInput;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);    
    private final CommandPS5Controller base = new CommandPS5Controller(0);
    private final CommandPS5Controller operator = new CommandPS5Controller(1);


    /* Drive Controls */
    private final int translationAxis = PS5Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS5Controller.Axis.kRightX.value;
    private final int rotationAxisY = PS5Controller.Axis.kRightY.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, PS5Controller.Button.kOptions.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final TransferSubsystem transfer = new TransferSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final LimelightSubsystem limelight = new LimelightSubsystem();

    /* Limelight */
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //calculating angle to speaker using Ll
    double limelightAngle = Math.atan((LimelightConstants.aprilTagToSpeakerHeight+LimelightConstants.heightToAprilTag)
    /(LimelightConstants.heightToAprilTag/Math.tan(LimelightHelpers.getTY("limelight"))))
    +10;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        /* Default Commands */
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                    s_Swerve,
                    limelight,
                    () -> -SquaredInput.scale(Constants.stickDeadband, driver.getRawAxis(translationAxis)),
                    () -> -SquaredInput.scale(Constants.stickDeadband, driver.getRawAxis(strafeAxis)),
                    () -> -SquaredInput.scale(Constants.stickDeadband, driver.getRawAxis(rotationAxis)),
                    () -> -SquaredInput.scale(Constants.stickDeadband, driver.getRawAxis(rotationAxisY)),
                    base.PS(), //robotCentricSup
                    operator.R1(), //passHeading
                    operator.circle(), //podiumHeading
                    operator.square(), //ampPassHeading
                    base.triangle(), //limelightTarget
                    base.triangle(), //povDown
                    base.R3())); //defenseMode
    
        intake.setDefaultCommand(intake.DefaultCommand());
        transfer.setDefaultCommand(transfer.DefaultCommand());
        shooter.setDefaultCommand(shooter.DefaultCommand());
        limelight.setDefaultCommand(limelight.DefaultCommand());
        
        /* Smart Dashboard */
        
        //Post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        SmartDashboard.putNumber("Arm Position", arm.currentArmPos());        
        SmartDashboard.putNumber("Left Shooter Speed", shooter.currentShooterSpeed()[0]);
        SmartDashboard.putNumber("Right Shooter Speed", shooter.currentShooterSpeed()[1]);
        SmartDashboard.putNumber("Limelight Arm Pos", limelight.limelightArmPos);
        SmartDashboard.putNumber("Limelight Angle", limelightAngle);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        base.options().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        base.R1().whileTrue(new SmartIntake(intake, transfer, arm));
        base.L1().whileTrue(new SmartOuttake(intake, transfer, arm));
        base.R2().whileTrue(new SimpleShoot(transfer, shooter));
        base.cross().onTrue(arm.SetArmToPos(ArmConstants.ampPos));
        base.touchpad().onTrue(arm.SetArmToPos(ArmConstants.subPos));
        base.triangle().onTrue(new AutoAim(limelight, arm));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}