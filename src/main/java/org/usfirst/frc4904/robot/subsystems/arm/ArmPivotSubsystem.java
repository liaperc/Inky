package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.motor.TelescopingArmPivotFeedForward;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    public static final double HARD_STOP_ARM_ANGLE = -38; // TODO make it stow angle
    public static final double HARD_STOP_BACK = 180 - HARD_STOP_ARM_ANGLE; // TODO make it stow angle

    public static final double GEARBOX_RATIO = 48; //48:1, 48 rotations of motor = 360 degrees
    public static final double GEARBOX_SLACK_DEGREES = 6;
    public static final double MAX_EXTENSION = Units.inchesToMeters(39.5);
    public static final double MIN_EXTENSION = 0;

    public static final double kS = 0;
    public static final double kV = 0.86;
    public static final double kA = 0.01;
    
    public static final double kG_retracted = 0.43;
    public static final double kG_extended = 1.08;

    // TODO: tune
    public static final double kP = 0.04;
    public static final double kI = 0.01;
    public static final double kD = 0;

    public final TalonMotorSubsystem armMotorGroup;
    public final TelescopingArmPivotFeedForward feedforward;
    public final DoubleSupplier extensionDealer;
    private final EncoderWithSlack slackyEncoder;

    public ArmPivotSubsystem(TalonMotorSubsystem armMotorGroup, DoubleSupplier extensionDealer) {
        this.armMotorGroup = armMotorGroup;
        this.extensionDealer = extensionDealer;
        this.feedforward = new TelescopingArmPivotFeedForward(kG_retracted, kG_extended, kS, kV, kA);
        this.slackyEncoder = new EncoderWithSlack(
            GEARBOX_SLACK_DEGREES,
            armMotorGroup::getSensorPositionRotations,
            Units.rotationsToDegrees(1/GEARBOX_RATIO),
            true
        );
    }

    public double getCurrentAngleDegrees() {
        // return slackyEncoder.getRealPosition();
        return motorRevsToAngle(armMotorGroup.getSensorPositionRotations());

    }

    /**
     * Expects sensors to be zeroed at forward hard-stop.
     */
    public void initializeEncoderPositions() {
        armMotorGroup.zeroSensors(angleToMotorRevs(HARD_STOP_ARM_ANGLE));
        slackyEncoder.zeroSlackDirection(true);
        armMotorGroup.configSoftwareLimits(angleToMotorRevs(HARD_STOP_ARM_ANGLE), angleToMotorRevs(HARD_STOP_BACK));
    }

    public static double motorRevsToAngle(double revs) {
        final double degrees_per_rotation = 360/GEARBOX_RATIO;
        final double degrees = revs * degrees_per_rotation;
        return degrees;
    }

    public static double angleToMotorRevs(double angle) {
        return angle / (360/GEARBOX_RATIO);
    }


    public Command c_controlAngularVelocity(DoubleSupplier degPerSecDealer) {
        return this.run(() -> {
            var ff = this.feedforward.calculate(
                1,//extensionDealer.getAsDouble()/MAX_EXTENSION,
                Units.degreesToRadians(getCurrentAngleDegrees()),
                Units.rotationsPerMinuteToRadiansPerSecond(Units.degreesToRotations(degPerSecDealer.getAsDouble()) * 60),
                0
            );
            SmartDashboard.putNumber("feedforward", ff);
            for (var motor: this.armMotorGroup.followMotors) {
                motor.setVoltage(ff);
            }
            this.armMotorGroup.leadMotor.setVoltage(ff);
        });
    }

    public Pair<Command, Double> c_holdRotation(double degreesFromHorizontal, double maxVelDegPerSec, double maxAccelDegPerSecSquare) {
        ezControl controller = new ezControl(
            kP, kI, kD,
            (position, velocityDegPerSec) -> { 
                double brr =  this.feedforward.calculate(
                    extensionDealer.getAsDouble()/MAX_EXTENSION, // TODO test if worky
                    Units.degreesToRadians(getCurrentAngleDegrees()),
                    Units.degreesToRadians(velocityDegPerSec),
                    0
                );
                SmartDashboard.putNumber("Intended voltage", maxAccelDegPerSecSquare);
                return brr;
            }
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelDegPerSec, maxAccelDegPerSecSquare),
            new TrapezoidProfile.State(degreesFromHorizontal, 0),
            new TrapezoidProfile.State(getCurrentAngleDegrees(), 0)
        );

        // return new Pair<Command,Double>(new ezMotion(controller, () -> this.getCurrentAngleDegrees() * Math.PI / 180, armMotorGroup::setVoltage,
        //         (double t) ->  new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity), this), profile.totalTime());
        return new Pair<Command,Double>(
            new ezMotion(
                controller,
                () -> this.getCurrentAngleDegrees(),
                (volts) -> {
                    // this.armMotorGroup.setVoltage(volts); // FIXME : use motorgroup setvoltage
                    SmartDashboard.putNumber("Arm Volts", volts);
                    this.armMotorGroup.leadMotor.setVoltage(volts);
                    for (var m : this.armMotorGroup.followMotors) m.setVoltage(volts);
                },
                (double t) -> {
                    SmartDashboard.putNumber("deg setpoint", profile.calculate(t).position);
                    SmartDashboard.putNumber("deg velocity", profile.calculate(t).velocity);
                    return new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity);
                },
            this, armMotorGroup), profile.totalTime());
    }
}
