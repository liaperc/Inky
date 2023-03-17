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
		var cmd2 = RobotMap.Component.intake.c_holdVoltage(4);
		cmd2.setName("Intake - manual intake activation");
		var cmdnull = RobotMap.Component.intake.c_holdVoltage(0);
		cmdnull.setName("Intake - deactivated");
		RobotMap.HumanInput.Operator.joystick.button2.onTrue(cmd2);
        RobotMap.HumanInput.Operator.joystick.button2.onFalse(cmdnull);

		// Outtake
		var cmd1 = RobotMap.Component.intake.c_holdVoltage(-4);
		cmd1.setName("Intake - manual outtake activation");
		RobotMap.HumanInput.Operator.joystick.button1.onTrue(cmd1);
        RobotMap.HumanInput.Operator.joystick.button1.onFalse(cmdnull);

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
