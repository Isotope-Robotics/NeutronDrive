package com.neutrondrive.math;

/** Unit conversion utilities for swerve drive calculations. */
public class Conversions {

    /**
     * @param wheelRPS    Wheel velocity in Rotations per Second
     * @param circumference Wheel circumference in meters
     * @return Wheel velocity in Meters per Second
     */
    public static double RPSToMPS(double wheelRPS, double circumference) {
        return wheelRPS * circumference;
    }

    /**
     * @param wheelMPS    Wheel velocity in Meters per Second
     * @param circumference Wheel circumference in meters
     * @return Wheel velocity in Rotations per Second
     */
    public static double MPSToRPS(double wheelMPS, double circumference) {
        return wheelMPS / circumference;
    }

    /**
     * @param wheelRotations Wheel position in Rotations
     * @param circumference  Wheel circumference in meters
     * @return Wheel distance in Meters
     */
    public static double rotationsToMeters(double wheelRotations, double circumference) {
        return wheelRotations * circumference;
    }

    /**
     * @param wheelMeters   Wheel distance in Meters
     * @param circumference Wheel circumference in meters
     * @return Wheel position in Rotations
     */
    public static double metersToRotations(double wheelMeters, double circumference) {
        return wheelMeters / circumference;
    }
}
