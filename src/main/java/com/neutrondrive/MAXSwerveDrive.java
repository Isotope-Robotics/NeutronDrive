package com.neutrondrive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * NeutronDrive swerve drivetrain subsystem for REV MAXSwerve modules.
 *
 * <p>Usage in your robot project:
 * <pre>{@code
 * MAXSwerveConfig config = new MAXSwerveConfig.Builder()
 *     .pigeonID(1)
 *     .trackWidth(Units.inchesToMeters(26.5))
 *     .wheelBase(Units.inchesToMeters(26.5))
 *     .maxSpeed(4.8)
 *     .drivingMotorPinionTeeth(14)
 *     .modules(
 *         new MAXSwerveModuleConstants(11, 10, -Math.PI / 2),  // Front Left
 *         new MAXSwerveModuleConstants(15, 14,  0),             // Front Right
 *         new MAXSwerveModuleConstants(13, 12,  Math.PI),       // Back Left
 *         new MAXSwerveModuleConstants(17, 16,  Math.PI / 2)   // Back Right
 *     )
 *     .build();
 *
 * MAXSwerveDrive drive = new MAXSwerveDrive(config);
 * }</pre>
 */
public class MAXSwerveDrive extends SubsystemBase {

    private final MAXSwerveConfig config;
    private final MAXSwerveModule[] modules;
    private final Pigeon2 gyro;
    private final SwerveDriveOdometry odometry;

    /**
     * Construct the MAXSwerve drivetrain from a fully built {@link MAXSwerveConfig}.
     */
    public MAXSwerveDrive(MAXSwerveConfig config) {
        this.config = config;
        REVConfigs revConfigs = new REVConfigs(config);

        gyro = new Pigeon2(config.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0.0);

        modules = new MAXSwerveModule[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new MAXSwerveModule(i, config.modules[i], revConfigs);
        }

        odometry = new SwerveDriveOdometry(config.kinematics, getGyroYaw(), getModulePositions());
    }

    /**
     * Drive the robot.
     *
     * @param translation   Desired XY velocity (m/s) as a Translation2d
     * @param rotation      Desired rotation rate (rad/s)
     * @param fieldRelative True = field-relative, false = robot-relative
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getHeading())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        SwerveModuleState[] states = config.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, config.maxSpeedMetersPerSecond);

        for (MAXSwerveModule mod : modules) {
            mod.setDesiredState(states[mod.moduleNumber]);
        }
    }

    /**
     * Directly set each module's state (used by autonomous trajectory following).
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, config.maxSpeedMetersPerSecond);
        for (MAXSwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }
    }

    /** @return Current state of each module (speed + angle). */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (MAXSwerveModule mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /** @return Current position of each module (distance + angle). */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (MAXSwerveModule mod : modules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /** @return Robot pose from odometry. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** Reset odometry to a known pose. */
    public void setPose(Pose2d pose) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    /** @return Robot heading from odometry. */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /** Snap the odometry heading to a specific angle without moving. */
    public void setHeading(Rotation2d heading) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(),
            new Pose2d(getPose().getTranslation(), heading));
    }

    /** Zero the odometry heading. */
    public void zeroHeading() {
        odometry.resetPosition(getGyroYaw(), getModulePositions(),
            new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /** @return Raw gyro yaw (not affected by odometry resets). */
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    /**
     * Lock all wheels in an X formation to resist being pushed.
     * Useful for holding position at the end of a match.
     */
    public void setX() {
        modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /** Reset all drive encoder positions to zero. */
    public void resetEncoders() {
        for (MAXSwerveModule mod : modules) {
            mod.resetEncoders();
        }
    }

    @Override
    public void periodic() {
        odometry.update(getGyroYaw(), getModulePositions());

        for (MAXSwerveModule mod : modules) {
            SmartDashboard.putNumber("MAXMod " + mod.moduleNumber + " Angle",    mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("MAXMod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
