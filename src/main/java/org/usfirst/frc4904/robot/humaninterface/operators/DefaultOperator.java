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
		var joystick = RobotMap.HumanInput.Operator.joystick;
		
        joystick.button3.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> -0.3));
        joystick.button3.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));

        joystick.button5.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0.3));
		joystick.button5.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));

		// Intake
		// FIXME: use nameCommand to make it cleaner with expresions (no varibales)
		var zeroIntake = RobotMap.Component.intake.c_holdVoltage(0);
		var holdPiece = RobotMap.Component.intake.c_holdVoltage(-2).withTimeout(0.5).andThen(RobotMap.Component.intake.c_holdVoltage(-1));
		var runIntake = RobotMap.Component.intake.c_holdVoltage(-8);
		var runOuttake = RobotMap.Component.intake.c_holdVoltage(3);

		// intake
		joystick.button2.onTrue(runIntake);
		joystick.button2.onFalse(holdPiece);

		// outtake
		joystick.button1.onTrue(runOuttake);
		joystick.button1.onFalse(zeroIntake);


		// position + place cube
		joystick.button7.onTrue(RobotMap.Component.arm.c_shootCubes(3));
		joystick.button9.onTrue(RobotMap.Component.arm.c_shootCubes(2));

		// position cone
		joystick.button8.onTrue(RobotMap.Component.arm.c_shootCones(3));
		joystick.button10.onTrue(RobotMap.Component.arm.c_shootCones(2));

		// intake positions
		joystick.button6.onTrue(RobotMap.Component.arm.c_posIntakeShelf());
		joystick.button4.onTrue(RobotMap.Component.arm.c_posIntakeFloor());

		// stow positions
		joystick.button11.onTrue(RobotMap.Component.arm.c_posReturnToHomeDown());
		joystick.button12.onTrue(RobotMap.Component.arm.c_posReturnToHomeUp());
	}
	
}
