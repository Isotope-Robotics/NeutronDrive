package com.neutrondrive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A single swerve module backed by two SparkMax motor controllers and a
 * REV Through Bore Encoder (V2) used as the absolute turning encoder.
 *
 * <p>Drive motor uses a relative encoder in velocity closed-loop mode.
 * Turn motor uses the absolute encoder in position closed-loop mode with
 * continuous wrap enabled so the PID always takes the shortest path.
 */
public class MAXSwerveModule {

    /** Index of this module (0 = FL, 1 = FR, 2 = BL, 3 = BR). */
    public final int moduleNumber;

    private final SparkMax drivingSpark;
    private final SparkMax turningSpark;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingController;
    private final SparkClosedLoopController turningController;

    private final double chassisAngularOffset;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * @param moduleNumber Index of this module (0–3)
     * @param constants    Per-module CAN IDs and chassis angular offset
     * @param revConfigs   Shared SparkMax configurations
     */
    public MAXSwerveModule(int moduleNumber, MAXSwerveModuleConstants constants, REVConfigs revConfigs) {
        this.moduleNumber = moduleNumber;
        this.chassisAngularOffset = constants.chassisAngularOffset;

        drivingSpark = new SparkMax(constants.driveMotorID, MotorType.kBrushless);
        turningSpark = new SparkMax(constants.turnMotorID, MotorType.kBrushless);

        drivingEncoder = drivingSpark.getEncoder();
        turningEncoder = turningSpark.getAbsoluteEncoder();

        drivingController = drivingSpark.getClosedLoopController();
        turningController = turningSpark.getClosedLoopController();

        // Reset to known state then persist so settings survive a power cycle
        drivingSpark.configure(revConfigs.drivingConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningSpark.configure(revConfigs.turningConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    /**
     * Apply a desired state to this module.
     *
     * <p>The chassis angular offset is applied before optimization so the PID
     * reference is in the encoder's native frame, while reported states are
     * always chassis-relative.
     *
     * @param desiredState Target speed (m/s) and angle (chassis-relative)
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Shift desired angle into the encoder's reference frame
        SwerveModuleState correctedState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))
        );

        // Optimize to avoid spinning more than 90 degrees
        correctedState.optimize(new Rotation2d(turningEncoder.getPosition()));

        drivingController.setSetpoint(correctedState.speedMetersPerSecond, ControlType.kVelocity);
        turningController.setSetpoint(correctedState.angle.getRadians(), ControlType.kPosition);

        this.desiredState = desiredState;
    }

    /**
     * @return Current measured state (speed in m/s + chassis-relative angle).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            drivingEncoder.getVelocity(),
            new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset)
        );
    }

    /**
     * @return Current position (distance traveled in meters + chassis-relative angle).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset)
        );
    }

    /** Reset the drive encoder position to zero. */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }
}
