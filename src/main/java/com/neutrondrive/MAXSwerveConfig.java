package com.neutrondrive;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Full configuration for a NeutronDrive MAXSwerve drivetrain.
 *
 * <p>Create one instance of this in your robot project and pass it to {@link MAXSwerveDrive}.
 * Use the {@link Builder} for a readable setup:
 *
 * <pre>{@code
 * MAXSwerveConfig config = new MAXSwerveConfig.Builder()
 *     .pigeonID(1)
 *     .trackWidth(Units.inchesToMeters(26.5))
 *     .wheelBase(Units.inchesToMeters(26.5))
 *     .maxSpeed(4.8)
 *     .drivingMotorPinionTeeth(14)
 *     .modules(
 *         new MAXSwerveModuleConstants(11, 10, -Math.PI / 2),
 *         new MAXSwerveModuleConstants(15, 14,  0),
 *         new MAXSwerveModuleConstants(13, 12,  Math.PI),
 *         new MAXSwerveModuleConstants(17, 16,  Math.PI / 2)
 *     )
 *     .build();
 *
 * MAXSwerveDrive drive = new MAXSwerveDrive(config);
 * }</pre>
 */
public class MAXSwerveConfig {

    /** NEO free speed in RPM. */
    private static final double NEO_FREE_SPEED_RPM = 5676.0;

    // Hardware
    /** CAN ID of the Pigeon2 gyro. */
    public final int pigeonID;

    // Geometry
    /** Track width (left-to-right wheel distance) in meters. */
    public final double trackWidthMeters;

    /** Wheel base (front-to-back wheel distance) in meters. */
    public final double wheelBaseMeters;

    /** Pre-built kinematics object for FL, FR, BL, BR layout. */
    public final SwerveDriveKinematics kinematics;

    // Performance
    /** Maximum translational speed in meters per second. */
    public final double maxSpeedMetersPerSecond;

    /** Maximum rotational speed in radians per second. */
    public final double maxAngularVelocityRadPerSec;

    // Module hardware
    /** Wheel diameter in meters (MAXSwerve default: 3 in = 0.0762 m). */
    public final double wheelDiameterMeters;

    /** Wheel circumference in meters. */
    public final double wheelCircumferenceMeters;

    /**
     * Overall drive gear reduction.
     * Calculated from pinion teeth: {@code (45 * 22) / (pinionTeeth * 15)}.
     */
    public final double drivingMotorReduction;

    /** Free-speed of the drive wheel in meters per second (used for feedforward). */
    public final double driveWheelFreeSpeedMPS;

    // Drive PID
    /** Drive velocity PID — proportional gain. */
    public final double driveKP;

    /** Drive velocity PID — integral gain. */
    public final double driveKI;

    /** Drive velocity PID — derivative gain. */
    public final double driveKD;

    // Turn PID
    /** Turn position PID — proportional gain. */
    public final double turnKP;

    /** Turn position PID — integral gain. */
    public final double turnKI;

    /** Turn position PID — derivative gain. */
    public final double turnKD;

    // Current limits
    /** Smart current limit for drive SparkMax (amps). */
    public final int driveCurrentLimit;

    /** Smart current limit for turn SparkMax (amps). */
    public final int turnCurrentLimit;

    // Idle modes
    /** Idle mode for drive SparkMax. */
    public final IdleMode driveIdleMode;

    /** Idle mode for turn SparkMax. */
    public final IdleMode turnIdleMode;

    // Per-module constants (FL, FR, BL, BR)
    /** Per-module CAN IDs and chassis angular offsets. */
    public final MAXSwerveModuleConstants[] modules;

    private MAXSwerveConfig(Builder b) {
        this.pigeonID = b.pigeonID;
        this.trackWidthMeters = b.trackWidthMeters;
        this.wheelBaseMeters = b.wheelBaseMeters;
        this.kinematics = new SwerveDriveKinematics(
            new Translation2d( wheelBaseMeters / 2.0,  trackWidthMeters / 2.0),
            new Translation2d( wheelBaseMeters / 2.0, -trackWidthMeters / 2.0),
            new Translation2d(-wheelBaseMeters / 2.0,  trackWidthMeters / 2.0),
            new Translation2d(-wheelBaseMeters / 2.0, -trackWidthMeters / 2.0)
        );
        this.maxSpeedMetersPerSecond = b.maxSpeedMetersPerSecond;
        this.maxAngularVelocityRadPerSec = b.maxAngularVelocityRadPerSec;
        this.wheelDiameterMeters = b.wheelDiameterMeters;
        this.wheelCircumferenceMeters = b.wheelDiameterMeters * Math.PI;
        // MAXSwerve gear ratio: 45 bevel teeth * 22 first-stage spur / (pinion teeth * 15 bevel pinion)
        this.drivingMotorReduction = (45.0 * 22.0) / (b.drivingMotorPinionTeeth * 15.0);
        double motorFreeSpeedRPS = NEO_FREE_SPEED_RPM / 60.0;
        this.driveWheelFreeSpeedMPS = (motorFreeSpeedRPS * wheelCircumferenceMeters) / drivingMotorReduction;
        this.driveKP = b.driveKP;
        this.driveKI = b.driveKI;
        this.driveKD = b.driveKD;
        this.turnKP = b.turnKP;
        this.turnKI = b.turnKI;
        this.turnKD = b.turnKD;
        this.driveCurrentLimit = b.driveCurrentLimit;
        this.turnCurrentLimit = b.turnCurrentLimit;
        this.driveIdleMode = b.driveIdleMode;
        this.turnIdleMode = b.turnIdleMode;
        this.modules = b.modules;
    }

    /** Builder for {@link MAXSwerveConfig}. */
    public static class Builder {
        private int pigeonID = 1;
        private double trackWidthMeters = 0.6731;  // 26.5 inches
        private double wheelBaseMeters = 0.6731;
        private double maxSpeedMetersPerSecond = 4.8;
        private double maxAngularVelocityRadPerSec = 2 * Math.PI;
        private double wheelDiameterMeters = 0.0762; // 3 inches
        private int drivingMotorPinionTeeth = 14;
        private double driveKP = 0.04, driveKI = 0.0, driveKD = 0.0;
        private double turnKP = 1.0, turnKI = 0.0, turnKD = 0.0;
        private int driveCurrentLimit = 50;
        private int turnCurrentLimit = 20;
        private IdleMode driveIdleMode = IdleMode.kBrake;
        private IdleMode turnIdleMode = IdleMode.kBrake;
        private MAXSwerveModuleConstants[] modules;

        /** CAN ID of the Pigeon2. */
        public Builder pigeonID(int id)                              { this.pigeonID = id; return this; }
        /** Track width (left-to-right) in meters. */
        public Builder trackWidth(double m)                          { this.trackWidthMeters = m; return this; }
        /** Wheel base (front-to-back) in meters. */
        public Builder wheelBase(double m)                           { this.wheelBaseMeters = m; return this; }
        /** Maximum translational speed in m/s. */
        public Builder maxSpeed(double mps)                          { this.maxSpeedMetersPerSecond = mps; return this; }
        /** Maximum rotational speed in rad/s. */
        public Builder maxAngularVelocity(double radps)              { this.maxAngularVelocityRadPerSec = radps; return this; }
        /** Wheel diameter in meters. */
        public Builder wheelDiameter(double m)                       { this.wheelDiameterMeters = m; return this; }
        /** MAXSwerve driving pinion tooth count (12, 13, or 14). */
        public Builder drivingMotorPinionTeeth(int teeth)            { this.drivingMotorPinionTeeth = teeth; return this; }
        /** Drive velocity PID kP. */
        public Builder driveKP(double v)                             { this.driveKP = v; return this; }
        /** Drive velocity PID kI. */
        public Builder driveKI(double v)                             { this.driveKI = v; return this; }
        /** Drive velocity PID kD. */
        public Builder driveKD(double v)                             { this.driveKD = v; return this; }
        /** Turn position PID kP. */
        public Builder turnKP(double v)                              { this.turnKP = v; return this; }
        /** Turn position PID kI. */
        public Builder turnKI(double v)                              { this.turnKI = v; return this; }
        /** Turn position PID kD. */
        public Builder turnKD(double v)                              { this.turnKD = v; return this; }
        /** Smart current limit for drive SparkMax. */
        public Builder driveCurrentLimit(int amps)                   { this.driveCurrentLimit = amps; return this; }
        /** Smart current limit for turn SparkMax. */
        public Builder turnCurrentLimit(int amps)                    { this.turnCurrentLimit = amps; return this; }
        /** Idle mode for drive SparkMax. */
        public Builder driveIdleMode(IdleMode m)                     { this.driveIdleMode = m; return this; }
        /** Idle mode for turn SparkMax. */
        public Builder turnIdleMode(IdleMode m)                      { this.turnIdleMode = m; return this; }
        /** Per-module constants (exactly 4, order: FL, FR, BL, BR). */
        public Builder modules(MAXSwerveModuleConstants... mods)     { this.modules = mods; return this; }

        /** Build and validate the config. */
        public MAXSwerveConfig build() {
            if (modules == null || modules.length != 4)
                throw new IllegalStateException("Exactly 4 MAXSwerveModuleConstants required");
            return new MAXSwerveConfig(this);
        }
    }
}
