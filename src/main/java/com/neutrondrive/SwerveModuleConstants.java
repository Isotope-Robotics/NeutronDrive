package com.neutrondrive;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Hardware constants for a single swerve module.
 *
 * <p>Create one of these for each corner of the robot (FL, FR, BL, BR):
 * <pre>{@code
 * SwerveModuleConstants frontLeft = new SwerveModuleConstants(1, 2, 1, Rotation2d.fromDegrees(0.0));
 * }</pre>
 */
public class SwerveModuleConstants {

    /** CAN ID of the drive TalonFX. */
    public final int driveMotorID;

    /** CAN ID of the angle (steer) TalonFX. */
    public final int angleMotorID;

    /** CAN ID of the CANcoder. */
    public final int cancoderID;

    /** Absolute angle offset of the CANcoder when the wheel points forward. */
    public final Rotation2d angleOffset;

    /**
     * @param driveMotorID  CAN ID of the drive TalonFX
     * @param angleMotorID  CAN ID of the angle TalonFX
     * @param cancoderID    CAN ID of the CANcoder
     * @param angleOffset   CANcoder offset (wheel-forward position)
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
    }
}
