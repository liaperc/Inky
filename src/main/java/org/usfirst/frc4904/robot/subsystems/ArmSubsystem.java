package org.usfirst.frc4904.robot.subsystems;

public class ArmSubsystem {
    public final PivotArmSubsystem pivotArmSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;
    public ArmSubsystem(PivotArmSubsystem pivotArmSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.pivotArmSubsystem = pivotArmSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }
    
    public void c_holdArmPose(double degreesFromHorizontal, double extensionLengthInches) {
        pivotArmSubsystem.c_holdRotation(degreesFromHorizontal);
        armExtensionSubsystem.c_holdExtension(extensionLengthInches);
    }


}
