package com.neutrondrive;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.neutrondrive.math.Conversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A single swerve module backed by a TalonFX drive motor, TalonFX angle motor, and CANcoder.
 */
public class SwerveModule {

    public final int moduleNumber;

    private final Rotation2d angleOffset;
    private final double wheelCircumference;
    private final double maxSpeed;

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedforward;

    // Phoenix 6 25.x: control requests take units-typed constructors
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(RotationsPerSecond.of(0));
    private final PositionVoltage anglePosition = new PositionVoltage(Rotations.of(0));

    /**
     * @param moduleNumber  Index of this module (0 = FL, 1 = FR, 2 = BL, 3 = BR)
     * @param constants     Per-module CAN IDs and angle offset
     * @param ctreConfigs   Shared CTRE motor configurations
     * @param config        Full drivetrain configuration
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants constants, CTREConfigs ctreConfigs, SwerveConfig config) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = constants.angleOffset;
        this.wheelCircumference = config.moduleType.wheelCircumference;
        this.maxSpeed = config.maxSpeedMetersPerSecond;
        this.driveFeedforward = new SimpleMotorFeedforward(config.driveKS, config.driveKV, config.driveKA);

        angleEncoder = new CANcoder(constants.cancoderID);
        angleEncoder.getConfigurator().apply(ctreConfigs.swerveCANcoderConfig);

        angleMotor = new TalonFX(constants.angleMotorID);
        angleMotor.getConfigurator().apply(ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        driveMotor = new TalonFX(constants.driveMotorID);
        driveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0.0);
    }

    /**
     * Apply a desired state to this module.
     *
     * @param desiredState  Target speed and angle
     * @param isOpenLoop    True = duty cycle control, false = velocity closed-loop
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // WPILib 2025: optimize() is now an instance method
        desiredState.optimize(getState().angle);
        angleMotor.setControl(anglePosition.withPosition(Rotations.of(desiredState.angle.getRotations())));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / maxSpeed;
            driveMotor.setControl(driveDutyCycle);
        } else {
            double rps = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, wheelCircumference);
            driveVelocity.Velocity = rps;
            driveVelocity.FeedForward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

    /** @return Absolute angle reported by the CANcoder. */
    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(
            angleEncoder.getAbsolutePosition().getValue().in(Rotations));
    }

    /** Reset the angle motor's internal encoder to match the CANcoder absolute position. */
    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        angleMotor.setPosition(absolutePosition);
    }

    /** @return Current measured state (speed + angle) of this module. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.getVelocity().getValue().in(RotationsPerSecond), wheelCircumference),
            Rotation2d.fromRotations(angleMotor.getPosition().getValue().in(Rotations))
        );
    }

    /** @return Current position (distance traveled + angle) of this module. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveMotor.getPosition().getValue().in(Rotations), wheelCircumference),
            Rotation2d.fromRotations(angleMotor.getPosition().getValue().in(Rotations))
        );
    }
}
