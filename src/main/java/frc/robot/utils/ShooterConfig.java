// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class ShooterConfig {
    private List<ShooterPreset> shooterConfigs;

    public ShooterConfig() {
        shooterConfigs = new ArrayList<ShooterPreset>();
    }

    public ShooterConfig(ArrayList<ShooterPreset> pShooterConfigs) {
        this.shooterConfigs = pShooterConfigs;
    }

    public List<ShooterPreset> getShooterConfigs() {
        return shooterConfigs;
    }
}