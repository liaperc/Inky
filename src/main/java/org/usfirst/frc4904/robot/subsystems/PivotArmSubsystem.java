package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motorcontrollers.TalonMotorController;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotArmSubsystem extends SubsystemBase {
    public final TalonMotorSubsystem armMotorGroup;
    public final static double GEARBOX_RATIO = 48; //48:1, 48 rotations of motor = 360 degrees
    public PivotArmSubsystem(TalonMotorSubsystem armMotorGroup) {
        this.armMotorGroup = armMotorGroup;
    }
    
    public TalonMotorSubsystem getTalonMotorSubsystem() {
        return armMotorGroup;
    }

    public Command c_holdRotation(double degreesFromHorizontal) {
        final double motor_rotation_per_degree = GEARBOX_RATIO/360;
        final double rotations = degreesFromHorizontal * motor_rotation_per_degree;
        return this.getTalonMotorSubsystem().c_holdPosition(rotations);
    }
}
