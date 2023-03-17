/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.util.function.DoubleSupplier;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.robot.seenoevil.RobotContainer2;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.humaninput.Driver;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;


public class Robot extends CommandRobotBase {
    private final RobotMap map = new RobotMap();
    private final RobotContainer2 donttouchme = new RobotContainer2(RobotMap.Component.frontLeftWheelTalon, RobotMap.Component.backLeftWheelTalon, RobotMap.Component.frontRightWheelTalon, RobotMap.Component.backRightWheelTalon, RobotMap.Component.navx);

    private final Driver driver = new NathanGain();
    private final org.usfirst.frc4904.standard.humaninput.Operator operator = new DefaultOperator();

    @Override
    public void initialize() {
        driverChooser.setDefaultOption(new NathanGain());
        operatorChooser.setDefaultOption(new DefaultOperator()); 
    }

    @Override
    public void teleopInitialize() {
        final double TURN_MULTIPLIER = 0.5;
        var cmd = RobotMap.Component.chassis.c_controlWheelVoltages(
                () -> new DifferentialDriveWheelVoltages(
                    (driver.getY() + TURN_MULTIPLIER * driver.getTurnSpeed()) * 12,
                    (driver.getY() - TURN_MULTIPLIER * driver.getTurnSpeed()) * 12
        ));
        cmd.setName("chassis - Teleop_Default - c_controlWheelVoltages");
        
        RobotMap.Component.chassis.setDefaultCommand(cmd);
        // Command gaming = RobotMap.Component.arm.armExtensionSubsystem.c_holdExtension(0.1, 0.1, 0.1).getFirst();
        // Command gaming = RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(10, 150, 200).getFirst();
        // gaming.schedule();

        // Command gaming2 = RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(() -> 0);
        // gaming2.schedule();

        // RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(() -> 0).schedule();
        // RobotMap.Component.chassis.setDefaultCommand(RobotMap.Component.chassis.c_controlChassisSpeedAndTurn(() -> new Pair<Double, Double>(Robot.drivingConfig.getX(), Robot.drivingConfig.getTurnSpeed())));
    }

    @Override
    public void teleopExecute() {
        // operator controller override

        System.out.println("button " + String.valueOf(RobotMap.HumanInput.Operator.joystick.button1.getAsBoolean())); // TODO: buttons
        final DoubleSupplier pivot_getter = () -> RobotMap.HumanInput.Operator.joystick.getAxis(1) * 30;
        final DoubleSupplier extension_getter = () -> RobotMap.HumanInput.Operator.joystick.getAxis(2) / 4;

        if (pivot_getter.getAsDouble() != 0) {
            var cmd = RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(pivot_getter::getAsDouble);
            cmd.setName("arm - Teleop - c_controlAngularVelocity");
            cmd.schedule();
        }
        if (extension_getter.getAsDouble() != 0) {
            var cmd = RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(extension_getter::getAsDouble);
            cmd.setName("arm - Teleop - c_controlVelocity");
            cmd.schedule();
        }
        
        SmartDashboard.putBoolean("isFlipped - IMPORTANT", NathanGain.isFlippy);
        SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());
        // System.out.println("gyro " + RobotMap.Component.navx.getAngle());
        // RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.setVoltage(2);

        RobotMap.HumanInput.Operator.joystick.button1.whileTrue(Commands.runOnce(() -> System.out.println("henoteuhnotheuntoheu")));
        
    }

    @Override
    public void autonomousInitialize() {
        RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Brake); 
        RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Brake); 
        RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Brake); 
        RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Brake); 

        final Trajectory backward = donttouchme.getTrajectory("straight_backward");
        // donttouchme.getTrajectory("straight_backward");
        final Trajectory forward = donttouchme.getTrajectory("straight_forward");
        // donttouchme.m_robotDrive.tankDriveVolts(5, 5);
        // var command = new SequentialCommandGroup(donttouchme.getAutonomousCommand(trajectory), donttouchme.getAutonomousCommand(trajectory2));
        
        
        //various autons, comment in what you want
        //var commnand = donttouchme.getAutonomousCommand(backward);
        // var commnand = donttouchme.notBalanceAuton();
        var commnand = donttouchme.balanceAuton(donttouchme.m_robotDrive::getWheelSpeeds, donttouchme.m_robotDrive::tankDriveVolts);
        commnand.schedule();
        // var command2 = donttouchme.getAutonomousCommand(trajectory2);
        // command2.andThen(command).schedule();
        // command.andThen(Commands.runOnce(() -> donttouchme.getAutonomousCommand(trajectory))).schedule();
    }

    @Override
    public void autonomousExecute() {
        SmartDashboard.putString("pose", donttouchme.m_robotDrive.getPose().toString());
    }

    @Override
    public void disabledInitialize() {
    }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
// RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Coast); 
//         RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Coast); 
//         RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Coast); 
//         RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Coast); 
//         RobotContainer2.Component.leftATalonFX.neutralOutput();
//         RobotContainer2.Component.leftBTalonFX.neutralOutput();
//         RobotContainer2.Component.rightATalonFX.neutralOutput();
//         RobotContainer2.Component.rightBTalonFX.neutralOutput();        
        
    }

    @Override
    public void testExecute() {
        RobotMap.Component.intake.setVoltage(5);
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Intake current left", RobotMap.Component.intake.leftMotors.leadMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake current right", RobotMap.Component.intake.rightMotors.leadMotor.getOutputCurrent());


        // RobotMap.Component.chassis.testFeedForward(0.5);
        // SmartDashboard.putNumber("backward vel ", RobotMap.Component.chassis.leftMotors.leadMotor.getSelectedSensorVelocity(0) /2048 / RobotMap.Metrics.Chassis.GEAR_RATIO * RobotMap.Metrics.Chassis.WHEEL_DIAMETER_METERS*Math.PI);
    }

    @Override
    public void alwaysExecute() {
        SmartDashboard.putNumber("Arm angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());
        SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());
    }

}

