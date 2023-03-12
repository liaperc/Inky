package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public static int DEFAULT_INTAKE_SPEED = -1; // TODO: set value
    public SparkMaxMotorSubsystem motors;
    public Intake (SparkMaxMotorSubsystem motors){ //motors has leftmotor and rightmotot
        this.motors = motors;
    }
    public Command setVoltage(double voltage) {
        return Commands.run(() -> motors.setVoltage(voltage));
    }

    public Command setVoltageDefault() {
        return Commands.run(() -> motors.setVoltage(DEFAULT_INTAKE_SPEED));
    }
}