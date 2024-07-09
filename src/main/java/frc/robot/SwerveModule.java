package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private double absolutePosition;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS,
            Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);
    public final CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        this.absolutePosition = moduleConstants.absolutePosition;

        // THIS IS ALSO A PROBLEM

        angleEncoder = new CANcoder(moduleConstants.cancoderID, "static");
        configCancoder();
        // angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "static");
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "static");
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    public void configCancoder() {
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        swerveCANcoderConfig.MagnetSensor.MagnetOffset = absolutePosition;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        angleEncoder.getConfigurator().apply(swerveCANcoderConfig);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                    Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        mAngleMotor.setPosition(angleEncoder.getAbsolutePosition().refresh().getValue());
        // double absolutePosition = getCANcoder().getRotations() -
        // angleOffset.getRotations();
        // angleOffset.getRotations();
        // mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(),
                        Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
    }
}