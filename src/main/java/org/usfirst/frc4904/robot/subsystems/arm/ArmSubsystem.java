package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.HashMap;

import org.opencv.core.Mat.Tuple2;

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
    
    public static final double MAX_VELOCITY_PIVOT = 0;
    public static final double MAX_ACCEL_PIVOT = 0;

    HashMap<Integer, Pair<Integer, Integer>> cubes = new HashMap<Integer, Pair<Integer, Integer>>();
    
    HashMap<Integer, Pair<Integer, Integer>> cones = new HashMap<Integer, Pair<Integer, Integer>>();

    public ArmSubsystem(ArmPivotSubsystem armPivotSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.armPivotSubsystem = armPivotSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
        this.cubes.put(1, new Pair<>(-33,0));
        this.cubes.put(2, new Pair<>(14,6));
        this.cubes.put(3, new Pair<>(22,28));
        this.cones.put(1, new Pair<>(-19,0));
        this.cones.put(2, new Pair<>(29,18));
        this.cones.put(3, new Pair<>(31,38));
    }
    public ParallelCommandGroup c_angleCubes(int shelf) {
        int degreesFromHorizontal = cubes.get(shelf).getFirst();
        int extensionLengthInches = cubes.get(shelf).getSecond();
        return c_holdArmPose(degreesFromHorizontal, extensionLengthInches);
    }
    public ParallelCommandGroup c_angleCones(int shelf) {
        int degreesFromHorizontal = cones.get(shelf).getFirst();
        int extensionLengthInches = cones.get(shelf).getSecond();
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
