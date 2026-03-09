package com.neutrondrive;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Full configuration for a NeutronDrive swerve drivetrain.
 *
 * <p>Create one instance of this in your robot project and pass it to {@link SwerveDrive}.
 * Use the {@link Builder} for a readable setup:
 *
 * <pre>{@code
 * SwerveConfig config = new SwerveConfig.Builder()
 *     .moduleType(TalonFXSwerveConstants.SDS.MK4i.Falcon500(TalonFXSwerveConstants.SDS.MK4i.driveRatios.L2))
 *     .trackWidth(Units.inchesToMeters(21.73))
 *     .wheelBase(Units.inchesToMeters(21.73))
 *     .pigeonID(1)
 *     .maxSpeed(4.5)
 *     .maxAngularVelocity(10.0)
 *     .driveKP(0.12).driveKS(0.32).driveKV(1.51).driveKA(0.27)
 *     .modules(Mod0.constants, Mod1.constants, Mod2.constants, Mod3.constants)
 *     .build();
 * }</pre>
 */
public class SwerveConfig {

    // Hardware
    public final int pigeonID;
    public final TalonFXSwerveConstants moduleType;

    // Geometry
    public final double trackWidthMeters;
    public final double wheelBaseMeters;
    public final SwerveDriveKinematics kinematics;

    // Performance
    public final double maxSpeedMetersPerSecond;
    public final double maxAngularVelocityRadPerSec;

    // Drive PID / FF
    public final double driveKP;
    public final double driveKI;
    public final double driveKD;
    public final double driveKS;
    public final double driveKV;
    public final double driveKA;

    // Ramp rates
    public final double openLoopRamp;
    public final double closedLoopRamp;

    // Neutral modes
    public final NeutralModeValue angleNeutralMode;
    public final NeutralModeValue driveNeutralMode;

    // Current limits
    public final int angleCurrentLimit;
    public final boolean angleEnableCurrentLimit;

    public final int driveCurrentLimit;
    public final boolean driveEnableCurrentLimit;

    // Per-module constants (FL, FR, BL, BR)
    public final SwerveModuleConstants[] modules;

    private SwerveConfig(Builder b) {
        this.pigeonID = b.pigeonID;
        this.moduleType = b.moduleType;
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
        this.driveKP = b.driveKP;
        this.driveKI = b.driveKI;
        this.driveKD = b.driveKD;
        this.driveKS = b.driveKS;
        this.driveKV = b.driveKV;
        this.driveKA = b.driveKA;
        this.openLoopRamp = b.openLoopRamp;
        this.closedLoopRamp = b.closedLoopRamp;
        this.angleNeutralMode = b.angleNeutralMode;
        this.driveNeutralMode = b.driveNeutralMode;
        this.angleCurrentLimit = b.angleCurrentLimit;
        this.angleEnableCurrentLimit = b.angleEnableCurrentLimit;
        this.driveCurrentLimit = b.driveCurrentLimit;
        this.driveEnableCurrentLimit = b.driveEnableCurrentLimit;
        this.modules = b.modules;
    }

    public static class Builder {
        private int pigeonID = 1;
        private TalonFXSwerveConstants moduleType;
        private double trackWidthMeters = 0.55;
        private double wheelBaseMeters = 0.55;
        private double maxSpeedMetersPerSecond = 4.5;
        private double maxAngularVelocityRadPerSec = 10.0;
        private double driveKP = 0.12, driveKI = 0.0, driveKD = 0.0;
        private double driveKS = 0.32, driveKV = 1.51, driveKA = 0.27;
        private double openLoopRamp = 0.25, closedLoopRamp = 0.0;
        private NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        private NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
        private int angleCurrentLimit = 25;
        private boolean angleEnableCurrentLimit = true;
        private int driveCurrentLimit = 35;
        private boolean driveEnableCurrentLimit = true;
        private SwerveModuleConstants[] modules;

        public Builder pigeonID(int id)                                  { this.pigeonID = id; return this; }
        public Builder moduleType(TalonFXSwerveConstants m)          { this.moduleType = m; return this; }
        public Builder trackWidth(double m)                              { this.trackWidthMeters = m; return this; }
        public Builder wheelBase(double m)                               { this.wheelBaseMeters = m; return this; }
        public Builder maxSpeed(double mps)                              { this.maxSpeedMetersPerSecond = mps; return this; }
        public Builder maxAngularVelocity(double rps)                    { this.maxAngularVelocityRadPerSec = rps; return this; }
        public Builder driveKP(double v)                                 { this.driveKP = v; return this; }
        public Builder driveKI(double v)                                 { this.driveKI = v; return this; }
        public Builder driveKD(double v)                                 { this.driveKD = v; return this; }
        public Builder driveKS(double v)                                 { this.driveKS = v; return this; }
        public Builder driveKV(double v)                                 { this.driveKV = v; return this; }
        public Builder driveKA(double v)                                 { this.driveKA = v; return this; }
        public Builder openLoopRamp(double v)                            { this.openLoopRamp = v; return this; }
        public Builder closedLoopRamp(double v)                          { this.closedLoopRamp = v; return this; }
        public Builder angleNeutralMode(NeutralModeValue m)              { this.angleNeutralMode = m; return this; }
        public Builder driveNeutralMode(NeutralModeValue m)              { this.driveNeutralMode = m; return this; }
        public Builder angleCurrentLimit(int a, boolean enable)          { this.angleCurrentLimit = a; this.angleEnableCurrentLimit = enable; return this; }
        public Builder driveCurrentLimit(int a, boolean enable)          { this.driveCurrentLimit = a; this.driveEnableCurrentLimit = enable; return this; }
        public Builder modules(SwerveModuleConstants... mods)            { this.modules = mods; return this; }

        public SwerveConfig build() {
            if (moduleType == null) throw new IllegalStateException("moduleType must be set");
            if (modules == null || modules.length != 4) throw new IllegalStateException("Exactly 4 SwerveModuleConstants required");
            return new SwerveConfig(this);
        }
    }
}
