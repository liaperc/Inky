package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.HashMap;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Intake;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ArmSubsystem extends SubsystemBase {
    public final ArmPivotSubsystem armPivotSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;

    public static final double MAX_VELOCITY_EXTENSION = 0;
    public static final double MAX_ACCEL_EXTENSION = 0;
    
    public static final double MAX_VELOCITY_PIVOT = 150;
    public static final double MAX_ACCEL_PIVOT = 200;

    
    public static final HashMap<Integer, Pair<Integer, Integer>> cones = new HashMap<Integer, Pair<Integer, Integer>>();
    static {
        cones.put(1, new Pair<>(-19,0));
        cones.put(2, new Pair<>(29,18));
        cones.put(3, new Pair<>(31,38));
    }
    public static final HashMap<Integer, Pair<Integer, Integer>> cubes = new HashMap<Integer, Pair<Integer, Integer>>();
    static {
        cubes.put(1, new Pair<>(-33,0));
        cubes.put(2, new Pair<>(14,6));
        cubes.put(3, new Pair<>(22,28));
    }


    public ArmSubsystem(ArmPivotSubsystem armPivotSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.armPivotSubsystem = armPivotSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }

    public Command c_angleCubes(int shelf) {
        int degreesFromHorizontal = cubes.get(shelf).getFirst();
        int extensionLengthInches = cubes.get(shelf).getSecond();

        if (NathanGain.isFlippy) {
            degreesFromHorizontal = (degreesFromHorizontal * -1) + 180;
        }

        var cmd = c_holdArmPose(degreesFromHorizontal, extensionLengthInches);
        cmd.setName("arm - c_angleCubes - " + shelf);
        return cmd;
    }
    public ParallelRaceGroup placeCube(int shelf, boolean reversed) {
        int degreesFromHorizontal = cubes.get(shelf).getFirst();
        if (reversed) {
            degreesFromHorizontal = (cubes.get(shelf).getFirst() * -1) + 180;
        }
        int extensionLengthInches = cubes.get(shelf).getSecond();

        return (c_holdArmPose(degreesFromHorizontal, extensionLengthInches)
        .alongWith(new WaitCommand(1).andThen(RobotMap.Component.intake.c_holdVoltage(-Intake.DEFAULT_INTAKE_VOLTS))
        )).withTimeout(5); //TODO: change timeout
    }
    public Command c_angleCones(int shelf) {
        int degreesFromHorizontal = cones.get(shelf).getFirst();
        int extensionLengthInches = cones.get(shelf).getSecond();
        
        if (NathanGain.isFlippy) {
            degreesFromHorizontal = (degreesFromHorizontal * -1) + 180;
        }

        var cmd = c_holdArmPose(degreesFromHorizontal, extensionLengthInches);
        cmd.setName("arm - c_angleCones - " + shelf);
        return cmd;
    }

    public ParallelCommandGroup c_resetAngleBottom(int shelf) {
        return c_holdArmPose(-38, 0);
    }


    public ParallelCommandGroup c_holdArmPose(double degreesFromHorizontal, double extensionLengthInches) {
        Command firstCommand;
        Command secondCommand;
        double wait;

        Pair<Command, Double> pivotMovement = armPivotSubsystem.c_holdRotation(degreesFromHorizontal, MAX_VELOCITY_PIVOT, MAX_ACCEL_PIVOT);
        Pair<Command, Double> extensionMovement = armExtensionSubsystem.c_holdExtension(extensionLengthInches, MAX_VELOCITY_EXTENSION, MAX_ACCEL_EXTENSION);

        if ((extensionLengthInches - armExtensionSubsystem.getCurrentExtensionLength()) > 0) {
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
                    (new SequentialCommandGroup(new WaitCommand(0.1), secondCommand)).schedule();
        // ((new WaitCommand(1)).andThen(secondCommand)).schedule();
        // secondCommand.schedule();      
        }); // long story. basically, parallel command group requires it's subcommands' requirements. however, we want one subcommand to be able to die wihle the other one lives, so we just do this instead and leak commands. it's fine because they'll get cleaned up when their atomic base subsystems gets taken over by new commands
        cmd.setName("arm - c_holdArmPose");
        return cmd;
    }
}
