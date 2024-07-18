package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.Swerve;
import frc.robot.utils.ShooterPreset;
import frc.robot.utils.VisionLookUpTable;

public class TeleopSwerve extends Command {

    private Swerve s_Swerve;
    private LimelightSubsystem limelight;
    private ArmSubsystem arm;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier rotationSupY;
    private BooleanSupplier robotCentricSup;
    // private BooleanSupplier passHeading;
    // private BooleanSupplier podiumHeading;
    // private BooleanSupplier ampPassHeading;
    // private BooleanSupplier farPodiumHeading;
    // private BooleanSupplier ampHeading;
    // private BooleanSupplier povDown;
    private BooleanSupplier limelightTarget;
    private BooleanSupplier limelightPivot;
    // private BooleanSupplier defenseMode;
    // private double heading;
    // private double desiredHeading;
    // private double offsetAngle;
    // private double speed;
    private double limelightPos;
    private VisionLookUpTable m_VisionLookUpTable;
    private ShooterPreset m_ShooterPreset;
    // private boolean fire = false;
    // private boolean fireDisable = false;

    // private double targetAngle = 0;

    private PIDController m_thetaController;

    // private static boolean limelightTracking = false;

    public TeleopSwerve(Swerve s_Swerve, LimelightSubsystem limelight, ArmSubsystem arm, 
            DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, 
            DoubleSupplier rotationSupY, BooleanSupplier robotCentricSup, BooleanSupplier passHeading, 
            BooleanSupplier podiumHeading, BooleanSupplier ampPassHeading, 
            BooleanSupplier limelightTarget, BooleanSupplier povDown, BooleanSupplier defenseMode,
            BooleanSupplier limelightPivot) {
        this.s_Swerve = s_Swerve;
        this.limelight = limelight;
        this.arm = arm;
        addRequirements(s_Swerve);  
        addRequirements(limelight);
        if(arm!=null) {addRequirements(arm);}

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.rotationSupY = rotationSupY;
        this.robotCentricSup = robotCentricSup;
        // pov buttons
        // this.passHeading = passHeading;
        // this.podiumHeading = podiumHeading;
        // this.ampPassHeading = ampPassHeading;
        this.limelightTarget = limelightTarget;
        this.limelightPivot = limelightPivot;
        // this.povDown = povDown;
        // this.defenseMode = defenseMode;
    }

    @Override
    public void initialize() {
        m_thetaController = new PIDController(0.00075, 0, 0.00075);
        m_thetaController.enableContinuousInput(-180, 180);
        m_VisionLookUpTable = new VisionLookUpTable();
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double rotationValY = MathUtil.applyDeadband(rotationSupY.getAsDouble(), Constants.stickDeadband);
        double rotationValX = rotationVal;

        m_thetaController.setP(Constants.headingPresetKp);
        m_thetaController.setD(Constants.headingPresetKd);
            
        if (Math.abs(rotationVal) + Math.abs(rotationValY) >0.1){
            m_thetaController.setP(Constants.driveHeadingKp);
            m_thetaController.setD(Constants.driveHeadingKd);
            m_thetaController.setSetpoint(Math.toDegrees(Math.atan2(rotationValX,rotationValY)));
            rotationVal = m_thetaController.calculate(
            (MathUtil.inputModulus(s_Swerve.getPose().getRotation().getDegrees(), -180, 180)),
            m_thetaController.getSetpoint());

            /*

            if(Math.abs(rotationVal) < 0.05)
            {
                rotationVal = 0;

            }*/
        }

        //source pass shot heading
        
        //limelight autoalign
        if(limelightTarget.getAsBoolean()) {
            rotationVal = limelight.limelight_aim_proportional();
        }

        //limelight autopivot
        if(limelightPivot.getAsBoolean()) {
            if (LimelightHelpers.getTV("limelight")) {
            m_ShooterPreset = m_VisionLookUpTable.getShooterPreset(
                limelight.detectedTargetDistance);
            double m_ArmPos = m_ShooterPreset.getArmAngle() * 56.1/90;
            arm.setArmPosition(MathUtil.clamp(m_ArmPos, ArmConstants.subPos, ArmConstants.ampPos));
            
            /*Old pivot code*/
            // limelightPos = limelight.setLimelightArmPos();
            // // System.out.println(limelightPos);
            // // SmartDashboard.putNumber("Limelight Pos", limelightPos);
            // arm.setArmPosition(MathUtil.clamp(limelightPos, ArmConstants.subPos, ArmConstants.ampPos));
            }
        }
        //defense mode

        rotationVal = MathUtil.clamp(rotationVal, -1,1);

        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true);

    }
}