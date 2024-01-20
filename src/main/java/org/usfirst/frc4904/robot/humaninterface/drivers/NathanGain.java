package org.usfirst.frc4904.robot.humaninterface.drivers;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.humaninput.Driver;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class NathanGain extends Driver {

	public static final double SPEED_EXP = 2;
	public static final double Y_SPEED_SCALE = 1;

	public static final double TURN_EXP = 4;
	public static final double TURN_SPEED_SCALE = 1;


	public static final double NORMAL_SPEED_GAIN = 0.5; // TODO TUNE
	public static final double NORMAL_TURN_GAIN = 0.25;

	public static final double PRECISE_SPEED_SCALE = 0.2;	
	public static final double PRECISE_TURN_SCALE = 0.1;

	// public static final double SPEEDY_TURN_GAIN = 0.7;

	public static double PRECISION_SCALE_Y = NORMAL_SPEED_GAIN;
	public static double PRECISION_SCALE_TURN = NORMAL_TURN_GAIN;

	public NathanGain() {
		super("NathanGain");
	}

	protected double scaleGain(double input, double gain, double exp) {
		return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
	}

	@Override
	public void bindCommands() {
		RobotMap.HumanInput.Driver.xbox.x().onTrue(new InstantCommand(
			() -> {
				NathanGain.PRECISION_SCALE_Y = PRECISE_SPEED_SCALE;
				NathanGain.PRECISION_SCALE_TURN = PRECISE_TURN_SCALE;
			}
		));
		RobotMap.HumanInput.Driver.xbox.x().onFalse(new InstantCommand((	
			) -> {
				NathanGain.PRECISION_SCALE_Y = NORMAL_SPEED_GAIN;
				NathanGain.PRECISION_SCALE_TURN = NORMAL_TURN_GAIN;
			}
		));
		
		RobotMap.HumanInput.Driver.xbox.y().onTrue(new InstantCommand(() -> NathanGain.PRECISION_SCALE_Y = 1));
		RobotMap.HumanInput.Driver.xbox.y().onFalse(new InstantCommand(() -> NathanGain.PRECISION_SCALE_TURN = NORMAL_SPEED_GAIN));
	}

	@Override
	public double getX() {
		return 0;
	}

	@Override
	public double getY() {
		double rawSpeed = RobotMap.HumanInput.Driver.xbox.getRightTriggerAxis() - RobotMap.HumanInput.Driver.xbox.getLeftTriggerAxis();
		double speed;
		
		speed = scaleGain(rawSpeed, NathanGain.PRECISION_SCALE_Y, NathanGain.SPEED_EXP) * NathanGain.Y_SPEED_SCALE;
	

		// double precisionDrive = scaleGain(RobotMap.HumanInput.Driver.xbox.getLeftY(), 0.08, 1.2);
		// double operatorDrive = scaleGain(-RobotMap.HumanInput.Operator.joystick.getAxis(1), 0.1, 1.2);
		
		return speed;// + precisionDrive;
	}

	@Override
	public double getTurnSpeed() {
		double rawTurnSpeed = RobotMap.HumanInput.Driver.xbox.getLeftX();
		double turnSpeed = scaleGain(rawTurnSpeed, NathanGain.PRECISION_SCALE_Y, NathanGain.TURN_EXP) * NathanGain.TURN_SPEED_SCALE;
		// double precisionTurnSpeed = scaleGain(RobotMap.HumanInput.Driver.xbox.getRightX(), 0.08, 1.2);
		// double operatorControlTurnSpeed = scaleGain(RobotMap.HumanInput.Operator.joystick.getAxis(0), 0.2, 1.5);
		
		return turnSpeed;// + precisionTurnSpeed;
	}
}
