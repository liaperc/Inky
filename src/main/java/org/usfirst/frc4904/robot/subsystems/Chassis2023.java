package org.usfirst.frc4904.robot.subsystems;


import org.usfirst.frc4904.standard.subsystems.chassis.WestCoastDrive;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;

import edu.wpi.first.wpilibj2.command.Command;

//FIXME: unused
public class Chassis2023 extends WestCoastDrive {
    public static final double RAMP_START_ANGLE = 9.0;
    public static final double RAMP_BALANCE_TOLERANCE = 4.0;
    public Chassis2023(double trackWidthMeters, double motorToWheelGearRatio, double wheelDiameterMeters,
    double drive_kP, double drive_kI, double drive_kD,
    AHRS navx, TalonMotorSubsystem leftMotorSubsystem, TalonMotorSubsystem rightMotorSubsystem) {
        super(trackWidthMeters, motorToWheelGearRatio, wheelDiameterMeters, drive_kP, drive_kI, drive_kD, navx, leftMotorSubsystem, rightMotorSubsystem);
    }
    // public Command c_followSpline(Trajectory trajectory, double ffks, double ffkv, double ffka,
    // double ramsete_b, double ramsete_zeta){
    //     RamseteCommand ramseteCommand = new RamseteCommand(
	// 	// RamseteCommand ramseteCommand = new RamseteCommand(
	// 		trajectory,
	// 		()->this.getPoseMeters(),
	// 		new RamseteController(ramsete_b, ramsete_zeta),
	// 		new SimpleMotorFeedforward(
	// 			ffks,
	// 			ffkv,
	// 			ffka),
	// 		this.kinematics,
	// 		()->this.getWheelSpeeds(),
	// 		new PIDController(0, 0, 0),
	// 		new PIDController(0, 0, 0),
	// 		// RamseteCommand passes volts to the callback
    //         (leftV, rightV) -> this.setWheelVoltages(leftV, rightV),
	// 		// (leftV, rightV) -> { this.leftMotors.leadMotor.setVoltage(leftV); this.rightMotors.leadMotor.setVoltage(rightV); },
	// 		this,
    //         this.leftMotors,
    //         this.rightMotors
    //         );
	
	// 	// Reset odometry to the starting pose of the trajectory.
	// 	Pose2d initialPose = trajectory.getInitialPose();
	// 	this.resetPoseMeters(initialPose);
	// 	SmartDashboard.putString("initial pose", initialPose.toString());
	// 	// return new Gaming(m_robotDrive);
	// 	// Run path following command, then stop at the end.
	// 	// return Commands.run(() -> m_robotDrive.tankDriveVolts(1, 1), m_robotDrive);
	// 	//return Commands.runOnce(() -> m_robotDrive.arcadeDrive(0.5, 0), m_robotDrive);
	// 	//return Commands.runOnce(() -> Component.testTalon.setVoltage(6));
	// 	return ramseteCommand;
	// 		//.andThen(() -> this.setWheelVoltages(0, 0));
    // }
	
    public Command c_driveUntilBalanced(AHRS gyro, double volts){
            return this.c_controlWheelVoltages(()->new DifferentialDriveWheelVoltages(volts,volts))
                .until(()-> Math.abs(gyro.getPitch()) > RAMP_START_ANGLE)
                .andThen(c_controlWheelVoltages(()->new DifferentialDriveWheelVoltages(volts,volts)))
                    .until(()-> Math.abs(gyro.getPitch()) < RAMP_BALANCE_TOLERANCE)
                .andThen(()->this.setWheelVoltages(0,0));
        }
    
    
}
