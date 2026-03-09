package com.neutrondrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/**
 * Builds CTRE motor/encoder configurations from a {@link SwerveConfig}.
 *
 * <p>Instantiated once inside {@link SwerveDrive} and shared with all modules.
 */
public class CTREConfigs {

    public final TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(SwerveConfig config) {
        TalonFXSwerveConstants mod = config.moduleType;

        // CANcoder
        swerveCANcoderConfig.MagnetSensor.SensorDirection = mod.cancoderInvert;

        // Angle motor
        swerveAngleFXConfig.MotorOutput.Inverted = mod.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = config.angleNeutralMode;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = mod.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = config.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = config.angleCurrentLimit;

        swerveAngleFXConfig.Slot0.kP = mod.angleKP;
        swerveAngleFXConfig.Slot0.kI = mod.angleKI;
        swerveAngleFXConfig.Slot0.kD = mod.angleKD;

        // Drive motor
        swerveDriveFXConfig.MotorOutput.Inverted = mod.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = config.driveNeutralMode;
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = mod.driveGearRatio;

        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = config.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = config.driveCurrentLimit;

        swerveDriveFXConfig.Slot0.kP = config.driveKP;
        swerveDriveFXConfig.Slot0.kI = config.driveKI;
        swerveDriveFXConfig.Slot0.kD = config.driveKD;

        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = config.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.closedLoopRamp;
    }
}
