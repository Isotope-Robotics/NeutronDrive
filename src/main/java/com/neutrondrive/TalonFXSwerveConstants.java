package com.neutrondrive;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/**
 * Pre-built constants for common COTS (Commercial Off-The-Shelf) swerve modules
 * that use TalonFX drive and steer motors.
 *
 * <p>Usage example:
 * <pre>{@code
 * TalonFXSwerveConstants moduleType =
 *     TalonFXSwerveConstants.SDS.MK4i.Falcon500(TalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);
 * }</pre>
 */
public class TalonFXSwerveConstants {

    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final InvertedValue driveMotorInvert;
    public final InvertedValue angleMotorInvert;
    public final SensorDirectionValue cancoderInvert;

    public TalonFXSwerveConstants(
            double wheelDiameter,
            double angleGearRatio,
            double driveGearRatio,
            double angleKP,
            double angleKI,
            double angleKD,
            InvertedValue driveMotorInvert,
            InvertedValue angleMotorInvert,
            SensorDirectionValue cancoderInvert) {
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.cancoderInvert = cancoderInvert;
    }

    // -------------------------------------------------------------------------
    // West Coast Products
    // -------------------------------------------------------------------------

    public static final class WCP {

        public static final class SwerveXStandard {
            public static TalonFXSwerveConstants Falcon500(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), (396.0 / 35.0), driveGearRatio,
                    1.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.Clockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static TalonFXSwerveConstants KrakenX60(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), (396.0 / 35.0), driveGearRatio,
                    30.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.Clockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static final class driveRatios {
                public static final double X1_10 = 7.85;
                public static final double X1_11 = 7.13;
                public static final double X1_12 = 6.54;
                public static final double X2_10 = 6.56;
                public static final double X2_11 = 5.96;
                public static final double X2_12 = 5.46;
                public static final double X3_12 = 5.14;
                public static final double X3_13 = 4.75;
                public static final double X3_14 = 4.41;
            }
        }

        public static final class SwerveXFlipped {
            public static TalonFXSwerveConstants Falcon500(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), (468.0 / 35.0), driveGearRatio,
                    1.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.Clockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static TalonFXSwerveConstants KrakenX60(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), (468.0 / 35.0), driveGearRatio,
                    30.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.Clockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static final class driveRatios {
                public static final double X1_10 = 8.10;
                public static final double X1_11 = 7.36;
                public static final double X1_12 = 6.75;
                public static final double X2_10 = 6.72;
                public static final double X2_11 = 6.11;
                public static final double X2_12 = 5.60;
                public static final double X3_10 = 5.51;
                public static final double X3_11 = 5.01;
                public static final double X3_12 = 4.59;
            }
        }
    }

    // -------------------------------------------------------------------------
    // Swerve Drive Specialties
    // -------------------------------------------------------------------------

    public static final class SDS {

        public static final class MK3 {
            public static TalonFXSwerveConstants Falcon500(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), 12.8, driveGearRatio,
                    1.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.CounterClockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static TalonFXSwerveConstants KrakenX60(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), 12.8, driveGearRatio,
                    30.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.CounterClockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static final class driveRatios {
                public static final double Standard = 8.16;
                public static final double Fast = 6.86;
            }
        }

        public static final class MK4 {
            public static TalonFXSwerveConstants Falcon500(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), 12.8, driveGearRatio,
                    1.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.CounterClockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static TalonFXSwerveConstants KrakenX60(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), 12.8, driveGearRatio,
                    30.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.CounterClockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static final class driveRatios {
                public static final double L1 = 8.14;
                public static final double L2 = 6.75;
                public static final double L3 = 6.12;
                public static final double L4 = 5.14;
            }
        }

        public static final class MK4i {
            public static TalonFXSwerveConstants Falcon500(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), (150.0 / 7.0), driveGearRatio,
                    100.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.Clockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static TalonFXSwerveConstants KrakenX60(double driveGearRatio) {
                return new TalonFXSwerveConstants(
                    Units.inchesToMeters(4.0), (150.0 / 7.0), driveGearRatio,
                    30.0, 0.0, 0.0,
                    InvertedValue.CounterClockwise_Positive,
                    InvertedValue.Clockwise_Positive,
                    SensorDirectionValue.CounterClockwise_Positive);
            }

            public static final class driveRatios {
                public static final double L1 = 8.14;
                public static final double L2 = 6.75;
                public static final double L3 = 6.12;
            }
        }
    }
}
