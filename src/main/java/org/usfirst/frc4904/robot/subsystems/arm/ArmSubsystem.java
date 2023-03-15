package org.usfirst.frc4904.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ArmSubsystem {
    public final ArmPivotSubsystem pivotArmSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;

    public static final double MAX_VELOCITY_EXTENSION = 0;
    public static final double MAX_ACCEL_EXTENSION = 0;
    
    public static final double MAX_VELOCITY_PIVOT = 0;
    public static final double MAX_ACCEL_PIVOT = 0;

    public ArmSubsystem(ArmPivotSubsystem pivotArmSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.pivotArmSubsystem = pivotArmSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }

    public ParallelCommandGroup c_holdArmPose(double degreesFromHorizontal, double extensionLengthInches) {
        var cmdgrp = new ParallelCommandGroup(
            pivotArmSubsystem.c_holdRotation(degreesFromHorizontal, MAX_VELOCITY_PIVOT, MAX_ACCEL_PIVOT).getFirst(),
            armExtensionSubsystem.c_holdExtension(extensionLengthInches, MAX_VELOCITY_EXTENSION, MAX_ACCEL_EXTENSION).getFirst()
        );
        cmdgrp.addRequirements(pivotArmSubsystem, armExtensionSubsystem);
        return cmdgrp;
    }


}
