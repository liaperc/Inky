package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.HashMap;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

        return c_holdArmPose(degreesFromHorizontal, extensionLengthInches);
    }
    public Command c_angleCones(int shelf) {
        int degreesFromHorizontal = cones.get(shelf).getFirst();
        int extensionLengthInches = cones.get(shelf).getSecond();
        
        if (NathanGain.isFlippy) {
            degreesFromHorizontal = (degreesFromHorizontal * -1) + 180;
        }

        return c_holdArmPose(degreesFromHorizontal, extensionLengthInches);
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

        var cmdgrp = new ParallelCommandGroup(
            firstCommand,
            new SequentialCommandGroup(new WaitCommand(wait * 1000), secondCommand)
        );
        
        cmdgrp.addRequirements(this);
        return cmdgrp;
    }


}
