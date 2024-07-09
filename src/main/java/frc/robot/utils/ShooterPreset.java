// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

public class ShooterPreset implements Comparable<ShooterPreset> {
    private double m_armAngle;
    private double m_leftShooter;
    private double m_rightShooter;
    private double m_distance;

    public ShooterPreset(double armAngle, double leftShooter, double rightShooter, double distance) {
        this.m_armAngle = armAngle;
        this.m_leftShooter = leftShooter;
        this.m_rightShooter = rightShooter;
        this.m_distance = distance;
    }

    public double getArmAngle() {
        return m_armAngle;
    }

    public double getLeftShooter() {
        return m_leftShooter;
    }

    public double getRightShooter() {
        return m_rightShooter;
    }

    public double getDistance() {
        return m_distance;
    }

    public void setArmAngle(double armAngle) {
        this.m_armAngle = armAngle;
    }

    public void setLeftShooter(double leftShooter) {
        this.m_leftShooter = leftShooter;
    }

    public void setRightShooter(double rightShooter) {
        this.m_rightShooter = rightShooter;
    }

    public void setDistance(double distance) {
        this.m_distance = distance;
    }

    @Override
    public int compareTo(ShooterPreset pPreset) {
        return Double.compare(this.getDistance(), pPreset.getDistance());
    }
}