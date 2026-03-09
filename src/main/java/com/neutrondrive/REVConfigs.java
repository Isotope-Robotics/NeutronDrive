package com.neutrondrive;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Builds REV SparkMax configurations from a {@link MAXSwerveConfig}.
 *
 * <p>Instantiated once inside {@link MAXSwerveDrive} and shared with all modules.
 */
public class REVConfigs {

    /** Shared SparkMax configuration for all driving motors. */
    public final SparkMaxConfig drivingConfig = new SparkMaxConfig();

    /** Shared SparkMax configuration for all turning motors. */
    public final SparkMaxConfig turningConfig = new SparkMaxConfig();

    /** @param config Fully built MAXSwerve drivetrain configuration */
    public REVConfigs(MAXSwerveConfig config) {
        // Driving: encoder outputs meters and meters/second after conversion
        double drivingFactor = config.wheelCircumferenceMeters / config.drivingMotorReduction;
        // kV in V/(m/s) — volts per unit of velocity at the encoder output
        double drivingVelocityFeedForward = 12.0 / config.driveWheelFreeSpeedMPS;

        // Turning: encoder outputs radians and radians/second after conversion
        double turningFactor = 2 * Math.PI;

        drivingConfig
            .idleMode(config.driveIdleMode)
            .smartCurrentLimit(config.driveCurrentLimit);
        drivingConfig.encoder
            .positionConversionFactor(drivingFactor)        // rotations -> meters
            .velocityConversionFactor(drivingFactor / 60.0); // RPM -> meters/second
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(config.driveKP, config.driveKI, config.driveKD)
            .outputRange(-1, 1)
            .feedForward.kV(drivingVelocityFeedForward);

        turningConfig
            .idleMode(config.turnIdleMode)
            .smartCurrentLimit(config.turnCurrentLimit);
        turningConfig.absoluteEncoder
            // Through Bore Encoder output shaft is opposite to the steering motor
            .inverted(true)
            .positionConversionFactor(turningFactor)        // rotations -> radians
            .velocityConversionFactor(turningFactor / 60.0) // RPM -> radians/second
            .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(config.turnKP, config.turnKI, config.turnKD)
            .outputRange(-1, 1)
            // Allow PID to wrap through 0 (e.g. 350° -> 10° goes through 0, not the long way)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);
    }
}
