package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.standard.humaninput.Operator;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DefaultOperator extends Operator {
	public DefaultOperator() {
		super("DefaultOperator");
	}

	public DefaultOperator(String name) {
		super(name);
	}

	@Override
	public void bindCommands() {
		
        RobotMap.HumanInput.Operator.joystick.button3.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> -0.3));
        RobotMap.HumanInput.Operator.joystick.button3.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));

        RobotMap.HumanInput.Operator.joystick.button5.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0.3));
        RobotMap.HumanInput.Operator.joystick.button5.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));
		

		// Flippy button
		// RobotMap.HumanInput.Operator.joystick.button12.onTrue(new InstantCommand(
		// 	() -> {
		// 		NathanGain.isFlippy = !NathanGain.isFlippy;
		// 	}
		// ));
		// don't use flippy because confusion

		// // position + place cube
		// RobotMap.HumanInput.Operator.joystick.button7.onTrue(RobotMap.Component.arm.placeCube(3));
		// RobotMap.HumanInput.Operator.joystick.button9.onTrue(RobotMap.Component.arm.placeCube(2));
		// RobotMap.HumanInput.Operator.joystick.button11.onTrue(RobotMap.Component.arm.placeCube(1));
		
		// //position cone
		// RobotMap.HumanInput.Operator.joystick.button8.onTrue(RobotMap.Component.arm.c_angleCones(3));
		// RobotMap.HumanInput.Operator.joystick.button10.onTrue(RobotMap.Component.arm.c_angleCones(2));
		// RobotMap.HumanInput.Operator.joystick.button12.onTrue(RobotMap.Component.arm.c_angleCones(1));

		// //shelf intake
		// RobotMap.HumanInput.Operator.joystick.button6.onTrue(RobotMap.Component.arm.c_posIntakeShelf());
		
		// //ground intake
		// RobotMap.HumanInput.Operator.joystick.button4.onTrue(RobotMap.Component.arm.c_posIntakeGround());
	}
	
}
