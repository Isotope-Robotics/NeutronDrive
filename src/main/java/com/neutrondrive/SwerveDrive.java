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
 * NeutronDrive swerve drivetrain subsystem.
 *
 * <p>Usage in your robot project:
 * <pre>{@code
 * SwerveConfig config = new SwerveConfig.Builder()
 *     .moduleType(TalonFXSwerveConstants.SDS.MK4i.Falcon500(TalonFXSwerveConstants.SDS.MK4i.driveRatios.L2))
 *     .trackWidth(Units.inchesToMeters(21.73))
 *     .wheelBase(Units.inchesToMeters(21.73))
 *     .pigeonID(1)
 *     .maxSpeed(4.5)
 *     .maxAngularVelocity(10.0)
 *     .driveKP(0.12).driveKS(0.32).driveKV(1.51).driveKA(0.27)
 *     .modules(
 *         new SwerveModuleConstants(1, 2, 1, Rotation2d.fromDegrees(0)),
 *         new SwerveModuleConstants(3, 4, 2, Rotation2d.fromDegrees(0)),
 *         new SwerveModuleConstants(5, 6, 3, Rotation2d.fromDegrees(0)),
 *         new SwerveModuleConstants(7, 8, 4, Rotation2d.fromDegrees(0))
 *     )
 *     .build();
 *
 * SwerveDrive drive = new SwerveDrive(config);
 * }</pre>
 */
public class SwerveDrive extends SubsystemBase {

    private final SwerveConfig config;
    private final CTREConfigs ctreConfigs;
    private final SwerveModule[] modules;
    private final Pigeon2 gyro;
    private final SwerveDriveOdometry odometry;

    /**
     * Construct the swerve drivetrain from a fully built {@link SwerveConfig}.
     */
    public SwerveDrive(SwerveConfig config) {
        this.config = config;
        this.ctreConfigs = new CTREConfigs(config);

        gyro = new Pigeon2(config.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0.0);

        modules = new SwerveModule[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModule(i, config.modules[i], ctreConfigs, config);
        }

        odometry = new SwerveDriveOdometry(config.kinematics, getGyroYaw(), getModulePositions());
    }

    /**
     * Drive the robot.
     *
     * @param translation   Desired XY velocity (m/s) as a Translation2d
     * @param rotation      Desired rotation rate (rad/s)
     * @param fieldRelative True = field-relative, false = robot-relative
     * @param isOpenLoop    True = duty-cycle control, false = velocity closed-loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getHeading())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        SwerveModuleState[] states = config.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, config.maxSpeedMetersPerSecond);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(states[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Directly set each module's state (used by autonomous trajectory following).
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, config.maxSpeedMetersPerSecond);
        for (SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /** @return Current state of each module (speed + angle). */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /** @return Current position of each module (distance + angle). */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
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

    /** Re-sync all module angle motors to their CANcoder absolute positions. */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : modules) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        odometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : modules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle",    mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
