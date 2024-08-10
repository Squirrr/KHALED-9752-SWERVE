package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.ArmConstants;
import frc.robot.Commands.AutoPivot;
import frc.robot.Commands.AutoPivotAndShoot;
import frc.robot.Commands.AutoShoot;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.SmartIntake;
import frc.robot.Commands.SmartOuttake;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.GateSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
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
    // private final CommandPS5Controller operator = new CommandPS5Controller(1);


    /* Drive Controls */
    private final int translationAxis = PS5Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS5Controller.Axis.kRightX.value;
    private final int rotationAxisY = PS5Controller.Axis.kRightY.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, PS5Controller.Button.kOptions.value);

    /* Subsystems */
    private final static Swerve s_Swerve = new Swerve();
    public final static TransferSubsystem transfer = new TransferSubsystem();
    private final static GateSubsystem gate = new GateSubsystem();
    public final static IntakeSubsystem intake = new IntakeSubsystem();
    private final static ShooterSubsystem shooter = new ShooterSubsystem();
    public final static ArmSubsystem arm = new ArmSubsystem();
    private final static LimelightSubsystem limelight = new LimelightSubsystem();
    private final static LEDSubsystem leds = new LEDSubsystem();
    
    private final SendableChooser<Command> autoChooser;

    /* Limelight */
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

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
                    base.povLeft(), //passHeading
                    base.circle(), //podiumHeading
                    () -> false, //ampPassHeading //base.square()
                    base.triangle(), //limelightTarget
                    base.povDown(), //povDown
                    base.R3())); //defenseMode
    
        intake.setDefaultCommand(intake.DefaultCommand());
        transfer.setDefaultCommand(transfer.DefaultCommand());
        shooter.setDefaultCommand(shooter.DefaultCommand());
        leds.setDefaultCommand(leds.defaultCommand());
        
        /* Smart Dashboard */


        //Post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        SmartDashboard.putNumber("Arm Position", arm.currentArmPos());        
        SmartDashboard.putNumber("Left Shooter Speed", shooter.currentShooterSpeed()[0]);
        SmartDashboard.putNumber("Right Shooter Speed", shooter.currentShooterSpeed()[1]);

        /* Named Commands */

        NamedCommands.registerCommand("LIMELIGHT_AUTOAIM", new ParallelDeadlineGroup(
            new WaitCommand(0.5),
            new TeleopSwerve(
                    s_Swerve, limelight,
                    () -> 0,() -> 0,() -> 0,() -> 0,
                    () -> false, //robotCentricSup
                    () -> false, //passHeading
                    () -> false, //podiumHeading
                    () -> false, //ampPassHeading
                    () -> true, //limelightTarget
                    () -> false, //povDown
                    () -> false))); //defenseMode

        NamedCommands.registerCommand("LIMELIGHT_AUTOPIVOT", new AutoPivot(arm, limelight));

        NamedCommands.registerCommand("LIMELIGHT_AUTOSHOOT", new ParallelRaceGroup(
            new WaitCommand(3),
            new AutoPivotAndShoot(arm, shooter, gate, transfer, limelight)
        ));

        NamedCommands.registerCommand("SMART_INTAKE", 
        new SmartIntake(intake, transfer, gate, arm));

        NamedCommands.registerCommand("STOP_INTAKE", new ParallelDeadlineGroup(
            new WaitCommand(0.1), intake.DefaultCommand()
        ));


        NamedCommands.registerCommand("SMART_TRANSFER", new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new WaitCommand(0.25), gate.Open()),
        new ParallelDeadlineGroup(
            new WaitCommand(0.5), transfer.Transfer(-0.1)), 
        new ParallelDeadlineGroup(
            new WaitCommand(0.5), transfer.Transfer(1)
        )));

        NamedCommands.registerCommand("STOP_TRANSFER", new ParallelDeadlineGroup(
            new WaitCommand(0.1), transfer.DefaultCommand()
        ));

        NamedCommands.registerCommand("START_SHOOTER", new ParallelDeadlineGroup(
            new WaitCommand(0.1), shooter.SpinShooters(2500, 3500)
        ));

        NamedCommands.registerCommand("SIMPLE_SHOOT", new ParallelDeadlineGroup(
            new WaitCommand(0.5), new ShootCommand(transfer, shooter, gate, 5000, 6000)
        ));

        NamedCommands.registerCommand("STOP_SHOOTER", new ParallelDeadlineGroup(
            new WaitCommand(0.1), shooter.DefaultCommand()
        ));

        /*Useless Commands */
        {
        // NamedCommands.registerCommand("SUB_PIVOT", new ParallelDeadlineGroup(
        //     new WaitCommand(0.5), arm.SetArmToPos(ArmConstants.subPos)
        // ));

        // NamedCommands.registerCommand("CENTER_PIVOT", new ParallelDeadlineGroup(
        //     new WaitCommand(0.5), arm.SetArmToPos(21)
        // ));

        // NamedCommands.registerCommand("LEFT_PIVOT", new ParallelDeadlineGroup(
        //     new WaitCommand(0.5), arm.SetArmToPos(25)
        // ));

        // NamedCommands.registerCommand("RIGHT_PIVOT", new ParallelDeadlineGroup(
        //     new WaitCommand(0.5), arm.SetArmToPos(23.5)
        // ));

        // NamedCommands.registerCommand("DUMB_INTAKE",
        //     new DumbIntake(intake, transfer, gate)
        // );
        // NamedCommands.registerCommand("SHOOTER_PREP", new ParallelDeadlineGroup(
        //     new WaitCommand(0.5), transfer.Transfer(-0.1)));
        // NamedCommands.registerCommand("OPEN_GATE", 
        // gate.Open());

        // NamedCommands.registerCommand("CLOSE_GATE", 
        // gate.Close());
        }
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
        base.R2().whileTrue(new SmartIntake(intake, transfer, gate, arm));
        base.L2().whileTrue(new SmartOuttake(intake, transfer, gate, arm));
        base.R1().whileTrue(new ShootCommand(transfer, shooter, gate, 5000, 6000));
        base.cross().onTrue(arm.SetArmToPos(ArmConstants.ampPos));
        base.touchpad().onTrue(arm.SetArmToPos(ArmConstants.subPos));


        /*Explaining this b/c it looks rly confusing
        * First, AutoPivot and AutoShoot are told to run in parallel
        * But AutoShoot consists of 3 parts: outtake a bit, spin up shooters, intake quickly
        * So we run the outtake a bit while we spin up shooters,
        * Then intake quickly after shooters are spun up
        */
        base.triangle().onTrue(new AutoPivot(arm, limelight)
        //Autoshoot
            .alongWith( new ParallelDeadlineGroup(
                new AutoShoot(shooter, limelight)),
                transfer.Transfer(-0.25).withTimeout(0.25)
                
            .andThen(transfer.Transfer(1).withTimeout(0.25))

        ));
     }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // The chosen routine will run in autonomous
        return autoChooser.getSelected();
    }
}
