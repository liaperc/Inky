package org.usfirst.frc4904.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ArmSubsystem {
    public final ArmPivotSubsystem pivotArmSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;

    public ArmSubsystem(ArmPivotSubsystem pivotArmSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.pivotArmSubsystem = pivotArmSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }

    public ParallelCommandGroup c_holdArmPose(double degreesFromHorizontal, double extensionLengthInches) {
        var cmdgrp = new ParallelCommandGroup(
            pivotArmSubsystem.c_holdRotation(degreesFromHorizontal, -1, -1),
            armExtensionSubsystem.c_holdExtension(extensionLengthInches, -1, -1)
        );
        cmdgrp.addRequirements(pivotArmSubsystem, armExtensionSubsystem);
        return cmdgrp;
    }


}
