// This is the subsystem for the arm extension. 
// Code by Russell from 4904

package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtension extends SubsystemBase {
    
    private final TalonMotorSubsystem motor;
    private final static double EXTENSION_SPEED = 0.5;
    private final static double SPOOL_DIAMETER = 1; // inches
    private final static double TOTAL_ARM_LENGTH = 71; //inches
    private final static double RETRACTED_ARM_LENGTH = 31; //inches
    public final static double SPOOL_CIRCUMFERENCE = Math.PI * SPOOL_DIAMETER; // Math.PI * SPOOL_DIAMETER
    private final static double GEARBOX_RATIO = 12; // 12:1 
    
    /**
     * Constructs a new ArmExtension subsystem.
     *
     * @param motor the motor controller used to extend the arm
     */
    public ArmExtension(TalonMotorSubsystem motor) {
        this.motor = motor;
    }
    
    /**
     * Returns the motor controller used to extend the arm.
     *
     * @return the motor controller used to extend the arm
     */
    public TalonMotorSubsystem getMotor() {
        return motor;
    }

    public double extensionToRotations(double extensionLengthInches) {
        final double spool_rotations = extensionLengthInches/SPOOL_CIRCUMFERENCE;
        double rotations = spool_rotations * GEARBOX_RATIO;
        return rotations;
    }
}

