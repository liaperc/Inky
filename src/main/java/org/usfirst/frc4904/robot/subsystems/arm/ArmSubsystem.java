package org.usfirst.frc4904.robot.subsystems.arm;

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

    public ArmSubsystem(ArmPivotSubsystem armPivotSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.armPivotSubsystem = armPivotSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }

    public Command c_holdArmPose(double degreesFromHorizontal, double extensionLengthInches) {
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

        return this.runOnce(() -> {
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
    }
}
