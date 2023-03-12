package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ArmSubsystem {
    public final PivotArmSubsystem pivotArmSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;
    public ArmSubsystem(PivotArmSubsystem pivotArmSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.pivotArmSubsystem = pivotArmSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }
    
    public ParallelCommandGroup c_holdArmPose(double degreesFromHorizontal, double extensionLengthInches) {
        var cmdgrp = new ParallelCommandGroup(
            pivotArmSubsystem.c_holdRotation(degreesFromHorizontal),
            armExtensionSubsystem.c_holdExtension(extensionLengthInches)
        );
        cmdgrp.addRequirements(this.pivotArmSubsystem, this.armExtensionSubsystem, this);
        return cmdgrp;
    }


}
