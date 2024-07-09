// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class SquaredInput {
    public static double scale(double deadbandLimit, double input) {
        double numeratorFactor = (Math.abs(input) - deadbandLimit);
        double denominatorFactor = 1 - deadbandLimit;
        double sign = (input >= 0 ? 1 : -1);

        return sign * ((numeratorFactor * numeratorFactor) / (denominatorFactor * denominatorFactor));
    }
}
