package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.HashMap;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Intake;
import org.usfirst.frc4904.standard.custom.Triple;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ArmSubsystem extends SubsystemBase {
    public final ArmPivotSubsystem armPivotSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;

    public static final double MAX_VELOCITY_EXTENSION = 1;
    public static final double MAX_ACCEL_EXTENSION = 2;
    
    public static final double MAX_VELOCITY_PIVOT = 150;
    public static final double MAX_ACCEL_PIVOT = 200;

    public static final double FUNNY_ANGLE_CORRECTION = 6;
    public static final double FUNNY_EXTENSION_CORRECTION = 0;

    public static final HashMap<Integer, Triple<Double, Double, Double>> shelfCones = new HashMap<>(); //in degrees, meters
    static {
        // cones.put(1, new Triple<>(-19., Units.inchesToMeters(0), 3.));
        shelfCones.put(2, new Triple<>(29. + FUNNY_ANGLE_CORRECTION, Units.inchesToMeters(19) + FUNNY_EXTENSION_CORRECTION, 3.2));
        shelfCones.put(3, new Triple<>(41. + FUNNY_ANGLE_CORRECTION, ArmExtensionSubsystem.MAX_EXTENSION_M + FUNNY_EXTENSION_CORRECTION, 3.2));
    }
    
    public static final HashMap<Integer, Triple<Double, Double, Double>> floorCones = new HashMap<>(); //in degrees, meters
    static {
        // cones.put(1, new Triple<>(-19., Units.inchesToMeters(0), 3.));
        floorCones.put(2, new Triple<>(29. + FUNNY_ANGLE_CORRECTION, Units.inchesToMeters(16) + FUNNY_EXTENSION_CORRECTION, 3.));
        floorCones.put(3, new Triple<>(31. + FUNNY_ANGLE_CORRECTION, ArmExtensionSubsystem.MAX_EXTENSION_M + FUNNY_EXTENSION_CORRECTION, 3.));
    }

    public static HashMap<Integer, Triple<Double, Double, Double>> cones = floorCones;

    public static final HashMap<Integer, Triple<Double, Double, Double>> cubes = new HashMap<>(); //in degrees, meters
    static {
        // cubes.put(1, new Triple<>(-33., Units.inchesToMeters(0), 3.));
        cubes.put(2, new Triple<>(15. + FUNNY_ANGLE_CORRECTION, Units.inchesToMeters(0), 4.5));
        cubes.put(3, new Triple<>(22. + FUNNY_ANGLE_CORRECTION, Units.inchesToMeters(0), 4.5));
    }

    public static final HashMap<String, Pair<Double, Double>> otherPositions = new HashMap<>();
    static {
        // https://docs.google.com/spreadsheets/d/1B7Ie4efOpuZb4UQsk8lHycGvi6BspnF74DUMLmiKGUM/edit#gid=0 in degrees, meters
        otherPositions.put("homeUp", new Pair<>(70., Units.inchesToMeters(0.))); // TODO: get number @thomasrimer
        otherPositions.put("homeDown", new Pair<>(-37., Units.inchesToMeters(0.)));
        otherPositions.put("intakeShelf", new Pair<>(31., Units.inchesToMeters(20.)));
    }


    public ArmSubsystem(ArmPivotSubsystem armPivotSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.armPivotSubsystem = armPivotSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }

    public Command c_posReturnToHomeUp() {
        var cmd = c_holdArmPose(otherPositions.get("homeUp").getFirst(), otherPositions.get("homeUp").getSecond()).getFirst();
        cmd.setName("arm position - home (up)");
        return cmd;
    }
    public Command c_posReturnToHomeDown() {
        var cmd = c_holdArmPose(otherPositions.get("homeDown").getFirst(), otherPositions.get("homeDown").getSecond()).getFirst();
        cmd.setName("arm position - home (down)");
        return cmd;
    }
    public Command c_posIntakeShelf() {
        // TODO: back up 14 inches -- remember to always use meters
        var cmd = c_holdArmPose(otherPositions.get("intakeShelf").getFirst(), otherPositions.get("intakeShelf").getSecond()).getFirst();
        cmd.setName("arm position - pre shelf intake");
        return cmd;
    }

    public Pair<Command, Double> c_angleCones(int shelf) {
        var degreesFromHorizontal = cones.get(shelf).getFirst();
        var extensionLengthMeters = cones.get(shelf).getSecond();

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);
    }

    public Command c_shootCones(int shelf) {
        var degreesFromHorizontal = cones.get(shelf).getFirst();
        var extensionLengthMeters = cones.get(shelf).getSecond();
        var voltage = cones.get(shelf).getThird();

        Pair<Command, Double> armMovement = c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);

        return armMovement.getFirst().withTimeout(armMovement.getSecond())
                .andThen(RobotMap.Component.intake.c_holdVoltage(voltage).withTimeout(0.5))
                .andThen(RobotMap.Component.intake.c_holdVoltage(0))
                .andThen(c_posReturnToHomeUp());
    }
   
    public Pair<Command, Double> c_angleCubes(int shelf) {
        var degreesFromHorizontal = cubes.get(shelf).getFirst();
        var extensionLengthMeters = cubes.get(shelf).getSecond();

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);
    }

    public Command c_shootCubes(int shelf) {
        var degreesFromHorizontal = cubes.get(shelf).getFirst();
        var extensionLengthMeters = cubes.get(shelf).getSecond();
        var voltage = cubes.get(shelf).getThird();

        Pair<Command, Double> armMovement = c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);

        return armMovement.getFirst().withTimeout(armMovement.getSecond())
                .andThen(RobotMap.Component.intake.c_holdVoltage(voltage).withTimeout(0.5))
                .andThen(RobotMap.Component.intake.c_holdVoltage(0))
                .andThen(c_posReturnToHomeUp());
    }

    public Pair<Command, Double> c_holdArmPose(double degreesFromHorizontal, double extensionLengthMeters) {
        // return this.runOnce(() -> System.out.println("TODO: hold arm pose crashes the code!"));
        Command firstCommand;
        Command secondCommand;
        double wait;

        Pair<Command, Double> pivotMovement = armPivotSubsystem.c_holdRotation(degreesFromHorizontal, MAX_VELOCITY_PIVOT, MAX_ACCEL_PIVOT);
        Pair<Command, Double> extensionMovement = armExtensionSubsystem.c_holdExtension(extensionLengthMeters, MAX_VELOCITY_EXTENSION, MAX_ACCEL_EXTENSION);

        if ((extensionLengthMeters - armExtensionSubsystem.getCurrentExtensionLength()) > 0) {
            firstCommand = pivotMovement.getFirst();
            wait = pivotMovement.getSecond();
            secondCommand = extensionMovement.getFirst();
        } else {
            firstCommand = extensionMovement.getFirst();
            wait = extensionMovement.getSecond();
            secondCommand = pivotMovement.getFirst();
        }
        var cmd = this.runOnce(() -> {
                    firstCommand.schedule();
                    // (Commands.run(() -> System.out.println(
                    //     "1: " + String.valueOf(firstCommand.isScheduled())
                    // + " ... 2: " + String.valueOf(secondCommand.isScheduled())
                    // + " ... cur :  " + armExtensionSubsystem.getCurrentCommand().getName() 
                    // + " ... joystick: " + String.valueOf(RobotMap.HumanInput.Operator.joystick.getAxis(3))
                    //     ))).schedule();
                    (new SequentialCommandGroup(new WaitCommand(wait), secondCommand)).schedule();
        // ((new WaitCommand(1)).andThen(secondCommand)).schedule();
        // secondCommand.schedule();      
        }); // long story. basically, parallel command group requires it's subcommands' requirements. however, we want one subcommand to be able to die wihle the other one lives, so we just do this instead and leak commands. it's fine because they'll get cleaned up when their atomic base subsystems gets taken over by new commands
        cmd.setName("arm - c_holdArmPose");
        return new Pair<Command, Double>(cmd, pivotMovement.getSecond() + extensionMovement.getSecond());
    }
}
