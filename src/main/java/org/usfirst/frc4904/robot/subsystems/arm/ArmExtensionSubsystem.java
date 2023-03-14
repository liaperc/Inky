// This is the subsystem for the arm extension. 
// Code by Russell from 4904

package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtensionSubsystem extends SubsystemBase {
    
    private final TalonMotorSubsystem motor;
    private final static double EXTENSION_SPEED = 0.5;
    private final static double SPOOL_DIAMETER = Units.inchesToMeters(1);
    public final static double SPOOL_CIRCUMFERENCE = Math.PI * SPOOL_DIAMETER; // Math.PI * SPOOL_DIAMETER
    private final static double GEARBOX_RATIO = 12; // 12:1 
    private final ArmFeedforward feedforward;
    private DoubleSupplier angleDealer;
    
    /**
     * Constructs a new ArmExtension subsystem.
     *
     * @param motor the motor controller used to extend the arm
     */
    public ArmExtensionSubsystem(TalonMotorSubsystem motor, DoubleSupplier angleDealer) {
        this.motor = motor;
        this.feedforward = new ArmFeedforward(1, 1, -1);
        this.angleDealer = angleDealer;

    }
    
    /**
     * Returns the motor controller used to extend the arm.
     *
     * @return the motor controller used to extend the arm
     */
    public TalonMotorSubsystem getMotor() {
        return motor;
    }

    public double getCurrentExtensionLength() {
        return revsToExtensionLength(motor.getSensorPositionRotations());
    }

    public double revsToExtensionLength(double rotations) {
        final double number_of_spool_rotations = rotations/GEARBOX_RATIO;
        final double extensionLength = number_of_spool_rotations * SPOOL_CIRCUMFERENCE;
        return extensionLength;
    }

    public Command c_holdExtension(double extensionLengthMeters, double maxVelocity, double maxAcceleration) {
        ezControl controller = new ezControl(1, 1, 1, 
                                            (double position, double velocity) -> this.feedforward.calculate(angleDealer.getAsDouble() + Math.PI/2, velocity, 0));
        
        TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), 
                                                        new TrapezoidProfile.State(extensionLengthMeters, 0), 
                                                        new TrapezoidProfile.State(getCurrentExtensionLength(), 0));

        return new ezMotion(controller, this::getCurrentExtensionLength, motor::setVoltage, (double t) -> new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity), this);
    }
}

