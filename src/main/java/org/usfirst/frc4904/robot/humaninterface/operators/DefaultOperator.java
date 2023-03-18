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
		// Intake
		var cmd2 = RobotMap.Component.intake.c_holdVoltage(-8);
		cmd2.setName("Intake - manual intake activation");
		var cmdnull = RobotMap.Component.intake.c_holdVoltage(0);
		cmdnull.setName("Intake - deactivated");
		RobotMap.HumanInput.Operator.joystick.button2.onTrue(cmd2);
        RobotMap.HumanInput.Operator.joystick.button2.onFalse(cmdnull);

		// Outtake
		var cmd1 = RobotMap.Component.intake.c_holdVoltage(3);
		cmd1.setName("Intake - manual outtake activation");
		RobotMap.HumanInput.Operator.joystick.button1.onTrue(cmd1);
        RobotMap.HumanInput.Operator.joystick.button1.onFalse(cmdnull);

		// Flippy button
		// RobotMap.HumanInput.Operator.joystick.button12.onTrue(new InstantCommand(
		// 	() -> {
		// 		NathanGain.isFlippy = !NathanGain.isFlippy;
		// 	}
		// ));
		//TODO: flip?
		// position + place cube
		RobotMap.HumanInput.Operator.joystick.button7.onTrue(RobotMap.Component.arm.placeCube(3));
		RobotMap.HumanInput.Operator.joystick.button9.onTrue(RobotMap.Component.arm.placeCube(2));
		RobotMap.HumanInput.Operator.joystick.button11.onTrue(RobotMap.Component.arm.placeCube(1));
		
		//position cone
		RobotMap.HumanInput.Operator.joystick.button8.onTrue(RobotMap.Component.arm.c_angleCones(3));
		RobotMap.HumanInput.Operator.joystick.button10.onTrue(RobotMap.Component.arm.c_angleCones(3));
		RobotMap.HumanInput.Operator.joystick.button12.onTrue(RobotMap.Component.arm.c_angleCones(3));

		//shelf intake
		RobotMap.HumanInput.Operator.joystick.button6.onTrue(RobotMap.Component.arm.c_posIntakeShelf());
		
		//ground intake
		RobotMap.HumanInput.Operator.joystick.button4.onTrue(RobotMap.Component.arm.c_posIntakeGround());


        // RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(
        //     () -> RobotMap.HumanInput.Operator.joystick.getAxis(1) / 4
        // ).schedule();
        // RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(
        //     () -> RobotMap.HumanInput.Operator.joystick.getAxis(2) * 0.5
        // ).schedule();
	}
	
}
