package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.standard.humaninput.Operator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DefaultOperator extends Operator {
	public DefaultOperator() {
		super("DefaultOperator");
	}

	public DefaultOperator(String name) {
		super(name);
	}

	@Override
	public void bindCommands() {
		// Intake
		RobotMap.HumanInput.Operator.joystick.button2.onTrue(RobotMap.Component.intake.c_holdVoltage(4));
        RobotMap.HumanInput.Operator.joystick.button2.onFalse(RobotMap.Component.intake.c_holdVoltage(0));

		// Outtake
		RobotMap.HumanInput.Operator.joystick.button1.onTrue(RobotMap.Component.intake.c_holdVoltage(-4));
        RobotMap.HumanInput.Operator.joystick.button1.onFalse(RobotMap.Component.intake.c_holdVoltage(0));

		// Flippy button
		RobotMap.HumanInput.Operator.joystick.button12.onTrue(new InstantCommand(
			() -> {
				NathanGain.isFlippy = !NathanGain.isFlippy;
			}
		));


        // RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(
        //     () -> RobotMap.HumanInput.Operator.joystick.getAxis(1) / 4
        // ).schedule();
        // RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(
        //     () -> RobotMap.HumanInput.Operator.joystick.getAxis(2) * 0.5
        // ).schedule();
	}
	
}
