package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public static int DEFAULT_INTAKE_VOLTS = 3;
    public SparkMaxMotorSubsystem leftMotors;
    public SparkMaxMotorSubsystem rightMotors;
    public Intake (SparkMaxMotorSubsystem leftMotor, SparkMaxMotorSubsystem rightMotor){ //motors has leftmotor and rightmotot
        leftMotors = leftMotor;
        rightMotors = rightMotor;

        //TODO: fix current limits
        rightMotors.leadMotor.setSmartCurrentLimit(55);
        leftMotors.leadMotor.setSmartCurrentLimit(55);
    }
    public void setVoltage(double voltage) {
        leftMotors.setVoltage(voltage);
        rightMotors.setVoltage(-1.3*voltage);
    }
    public void setPower(double power) {
        leftMotors.setPower(power);
        rightMotors.setPower(-1.3*power);
    }
    public Command c_holdVoltage(double voltage) {
        return Commands.run(() -> {
            setVoltage(voltage);
        }, leftMotors, rightMotors);
    }

    public Command c_holdVoltageDefault() {
        var cmd = Commands.run(() -> setVoltage(DEFAULT_INTAKE_VOLTS), leftMotors, rightMotors);
        cmd.setName("Intake - c_holdVoltageDefault");
        return cmd;
    }
}
