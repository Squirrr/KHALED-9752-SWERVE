// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Collections;
import java.util.List;

import frc.robot.Constants;

/** Add your docs here. */
public class VisionLookUpTable {
    /*Code from 9752worlds vvv*/
    ShooterConfig shooterConfig;

    private static VisionLookUpTable instance;
    private static VisionLookUpTable getInstance() {
        return instance;
    }

    public VisionLookUpTable() {
        shooterConfig = Constants.getLookupTable();
        Collections.sort(shooterConfig.getShooterConfigs());
    }

    public ShooterPreset getShooterPreset(double DistanceFromTarget) {
        int endIndex = shooterConfig.getShooterConfigs().size() - 1;

        /*Gets the ShooterConfigs as an array, then gets the distance from the ShooterConfig
         *at index 0
        */
        if (DistanceFromTarget <= shooterConfig.getShooterConfigs().get(0).getDistance()) {
            return shooterConfig.getShooterConfigs().get(0);
        }
        
        if (DistanceFromTarget >= shooterConfig.getShooterConfigs().get(endIndex).getDistance()) {
            return shooterConfig.getShooterConfigs().get(endIndex);
        }

        return binarySearchDistance(shooterConfig.getShooterConfigs(), 0, endIndex, DistanceFromTarget);
    }
    /*Code from 9752worlds ^^^*/

    /*New Code*/
    private ShooterPreset binarySearchDistance(List<ShooterPreset> sortedArray, int low, int high, double key) {
        int index = Integer.MAX_VALUE;
    
        while (low <= high) {
            int mid = low  + ((high - low) / 2);

            if (sortedArray.get(mid).getDistance() < key) {
                low = mid + 1;
            } else if (sortedArray.get(mid).getDistance() > key) {
                high = mid - 1;
            } else if (sortedArray.get(mid).getDistance() == key) {
                index = mid;
                break;
            }
            // System.out.println("Low:" + low);
            // System.out.println("High:" + high);
        }

        if (low>high) {
            return interpolatedShooterPreset(sortedArray, high, low, key);
        }
        return sortedArray.get(index);
    }

    private ShooterPreset interpolatedShooterPreset(List<ShooterPreset> sortedArray, int low, int high, double key) {
        double highDistance = sortedArray.get(high).getDistance();
        double lowDistance = sortedArray.get(low).getDistance();

        double lowLeftShooterSpeed = sortedArray.get(low).getLeftShooter();
        double highLeftShooterSpeed = sortedArray.get(high).getLeftShooter();

        double lowRightShooterSpeed = sortedArray.get(low).getRightShooter();
        double highRightShooterSpeed = sortedArray.get(high).getRightShooter();

        double lowArmAngle = sortedArray.get(low).getArmAngle();
        double highArmAngle = sortedArray.get(high).getArmAngle();

        // System.out.println("low arm angle:" + lowArmAngle);
        // System.out.println("high arm angle:" + highArmAngle);

        double percentIn = (highDistance-key)/(highDistance-lowDistance);

        return new ShooterPreset(
            lowArmAngle+(highArmAngle-lowArmAngle)*percentIn,
            lowLeftShooterSpeed+(highLeftShooterSpeed-lowLeftShooterSpeed)*percentIn,
            lowRightShooterSpeed+(highRightShooterSpeed-lowRightShooterSpeed)*percentIn,
            lowDistance+(highDistance-lowDistance)*percentIn
        );


    }
}
