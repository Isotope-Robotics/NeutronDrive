package com.neutrondrive;

/**
 * Hardware constants for a single REV MAXSwerve module.
 *
 * <p>Create one of these for each corner of the robot (FL, FR, BL, BR):
 * <pre>{@code
 * MAXSwerveModuleConstants frontLeft = new MAXSwerveModuleConstants(11, 10, -Math.PI / 2);
 * }</pre>
 *
 * <p>Standard MAXSwerve chassis angular offsets (wheels pointing forward):
 * <ul>
 *   <li>Front Left:  {@code -Math.PI / 2}</li>
 *   <li>Front Right: {@code 0}</li>
 *   <li>Back Left:   {@code Math.PI}</li>
 *   <li>Back Right:  {@code Math.PI / 2}</li>
 * </ul>
 */
public class MAXSwerveModuleConstants {

    /** CAN ID of the driving SparkMax. */
    public final int driveMotorID;

    /** CAN ID of the turning SparkMax. */
    public final int turnMotorID;

    /** Chassis angular offset in radians (angle the wheel reads when pointing forward). */
    public final double chassisAngularOffset;

    /**
     * @param driveMotorID        CAN ID of the driving SparkMax
     * @param turnMotorID         CAN ID of the turning SparkMax
     * @param chassisAngularOffset Chassis angular offset in radians
     */
    public MAXSwerveModuleConstants(int driveMotorID, int turnMotorID, double chassisAngularOffset) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.chassisAngularOffset = chassisAngularOffset;
    }
}
