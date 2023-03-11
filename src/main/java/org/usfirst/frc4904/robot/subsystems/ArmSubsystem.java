package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motorcontrollers.TalonMotorController;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    public final TalonMotorSubsystem armMotorGroup;
    public ArmSubsystem(TalonMotorSubsystem armMotorGroup) {
        this.armMotorGroup = armMotorGroup;
    }
    
}
