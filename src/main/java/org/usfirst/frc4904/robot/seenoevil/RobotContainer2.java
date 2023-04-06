// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.seenoevil;

import static java.util.Map.entry;   
import static org.usfirst.frc4904.robot.Utils.nameCommand; 

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.usfirst.frc4904.robot.seenoevil.Constants.AutoConstants;
import org.usfirst.frc4904.robot.seenoevil.Constants.DriveConstants;
import org.usfirst.frc4904.robot.subsystems.arm.ArmSubsystem;
import org.usfirst.frc4904.standard.commands.TriggerCommandFactory;
import org.usfirst.frc4904.standard.custom.sensors.NavX;
import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.seenoevil.Balance;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer2 {
    private static final double placeholderconstant = 0; //TODO: add value
    // private static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics)
    //     // Apply the voltage constraint
    //     .addConstraint(new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter
    //         ),
    //         DriveConstants.kDriveKinematics,
    //         10
    //     ));
    // private static TrajectoryConfig trajectoryConfigReversed = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     .setKinematics(DriveConstants.kDriveKinematics)
    //     .addConstraint(new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         10))
    //     .setReversed(true);
    // private static TrajectoryConfig trajectoryConfigSlow = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond / 3,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2)
    //     .setKinematics(DriveConstants.kDriveKinematics)
    //     .addConstraint(new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         10))
    //     .setReversed(false);
    // private static TrajectoryConfig trajectoryConfigSlowReversed = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond / 3,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2)
    //     .setKinematics(DriveConstants.kDriveKinematics)
    //     .addConstraint(new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         10))
    //     .setReversed(true);

    private static BiFunction<Double, Double, TrajectoryConfig> fwdTrajectoryConfig = (speed, accel) -> new TrajectoryConfig(speed, accel)
        .setKinematics(DriveConstants.kDriveKinematics)
        .addConstraint(new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10));

    private static BiFunction<Double, Double, TrajectoryConfig> revTrajectoryConfig = (speed, accel) -> new TrajectoryConfig(speed, accel)
    .setKinematics(DriveConstants.kDriveKinematics)
    .addConstraint(new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10)).setReversed(true);

    private static double speed = AutoConstants.kMaxSpeedMetersPerSecond;
    private static double accel = AutoConstants.kMaxAccelerationMetersPerSecondSquared;

        private static Map<String, Trajectory> trajectories = Map.ofEntries(
                entry("sickle", TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        // List.of(new Translation2d(0.33*dist, .15*dist), new Translation2d(0.66*dist, -.15*dist)),
                        List.of(new Translation2d(1, -1), new Translation2d(2, -1)),
        
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        fwdTrajectoryConfig.apply(speed, accel))
                ),
                entry("straight_forward", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),//List.of(new Translation2d(2, 0)),
                        new Pose2d(2, 0, new Rotation2d(0)),
                        fwdTrajectoryConfig.apply(speed, accel)
                )),
                entry("straight_backward", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(Math.PI)),
                        List.of(),
                        new Pose2d(2, 0, new Rotation2d(Math.PI)),
                        revTrajectoryConfig.apply(speed, accel)
                )),
                entry("turn_right", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),
                        new Pose2d(1, -1, new Rotation2d(-Math.PI/2)),
                        fwdTrajectoryConfig.apply(speed, accel)
                )),
                entry("past_ramp", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),
                        new Pose2d(4, 0, new Rotation2d(0)),
                        fwdTrajectoryConfig.apply(speed, accel)
                )),
                entry("back_to_ramp", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(Math.PI)),
                        List.of(),
                        new Pose2d(1, 0, new Rotation2d(Math.PI)),
                        revTrajectoryConfig.apply(speed, accel)
                )),

        // new auton
        entry("to_ramp", TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(Units.inchesToMeters(24.19 - .5), 0, new Rotation2d(0)),
                fwdTrajectoryConfig.apply(speed, accel))),

        entry("go_over_ramp", TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(Units.inchesToMeters(118.02 - .5) + placeholderconstant, 0, new Rotation2d(0)),
                fwdTrajectoryConfig.apply(speed, accel))),
        entry("ramp_start", TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(Units.inchesToMeters(5.001) + placeholderconstant, 0, new Rotation2d(0)),
                fwdTrajectoryConfig.apply(speed/3, accel/2))),
        entry("angle_ramp_backward", TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                List.of(),
                new Pose2d(Units.inchesToMeters(5.001) + placeholderconstant, 0, new Rotation2d(Math.PI)),
                revTrajectoryConfig.apply(speed/3, accel/2))),
        entry("go_middle_ramp", TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                List.of(),
                new Pose2d(Units.inchesToMeters(53.5), 0, new Rotation2d(Math.PI)),
                revTrajectoryConfig.apply(speed, accel))),
        entry("go_to_pickup_next", TrajectoryGenerator.generateTrajectory( //from cone place to cube pickup
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(4.7625, 0.45085, new Rotation2d(0)), //x is 15 foot 7.5, y is 17.75 inches
                fwdTrajectoryConfig.apply(4., 3.))),
        entry("from_pickup_to_place", TrajectoryGenerator.generateTrajectory( //from cube pickup to cube node
                new Pose2d(0, 0, new Rotation2d(Math.PI)), 
                List.of(),//same x as last time, little extra is to straighten out, could be tuned
                new Pose2d(4.7625+0.1, 0.15, new Rotation2d(Math.PI)),//y is 15 cm to get to the cube node
                revTrajectoryConfig.apply(4., 3.))),
        entry("from_cube_place_to_ramp_edge", TrajectoryGenerator.generateTrajectory( //very curvy, might not work if we cant physically turn fast enough
            new Pose2d(0, 0, new Rotation2d(0)), //y is 15 cm to get to the cube node
            List.of(),
            new Pose2d(0.503+0.4, 0.95, new Rotation2d(0)),//TODO: 0.4 is extra to get onto ramp, both needs tuning and we need to now how far we get onto ramp before stalling out 
            fwdTrajectoryConfig.apply(speed, accel))),//I chose 0.4 bc its the length of the first section of the ramp, but other values might be better
        entry("onto_ramp", TrajectoryGenerator.generateTrajectory( //balancing on ramp, NEEDS TUNING
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(), //chargestation has 2 parts, ramp and platform. 0.61 to balance form robot center at platform start
            new Pose2d(0.61+0.2, 0, new Rotation2d(0)),//TODO: 0.2 is extra depending on how far onto the ramp we get stuck -- NEEDS TUNING    
            fwdTrajectoryConfig.apply(speed, accel))));




    public static class Component {
        public static WPI_TalonFX leftATalonFX;
        public static WPI_TalonFX leftBTalonFX;
        public static WPI_TalonFX rightATalonFX;
        public static WPI_TalonFX rightBTalonFX;
        public static WPI_TalonFX testTalon;
    }

    // The robot's subsystems
    // motors

    public final DriveSubsystem m_robotDrive;
    // public final XboxController m_driverController;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer2(WPI_TalonFX leftATalonFX, WPI_TalonFX leftBTalonFX, WPI_TalonFX rightATalonFX,
            WPI_TalonFX rightBTalonFX, AHRS navx) {

        // The driver's controller
        // m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        // Configure the button bindings
        Component.leftATalonFX = leftATalonFX;
        Component.leftBTalonFX = leftBTalonFX;
        Component.rightATalonFX = rightATalonFX;
        Component.rightBTalonFX = rightBTalonFX;

        RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Coast);
        RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Coast);
        RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Coast);
        RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Coast);

        Component.testTalon = new WPI_TalonFX(1);
        // Component.leftATalonFX.setInverted(true);
        // Component.leftBTalonFX.setInverted(true);

        this.m_robotDrive = new DriveSubsystem(navx);

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        // // A split-stick arcade command, with forward/backward controlled by the left
        // // hand, and turning controlled by the right.
        // new RunCommand(
        // () -> m_robotDrive.arcadeDrive(
        // -m_driverController.getLeftY(), -m_driverController.getRightX()),
        // m_robotDrive));
    }

    // public Command getAutonomousCommand(Trajectory trajectory) {
    public Command getAutonomousCommand(String trajectoryName) {
        final Trajectory trajectory = getTrajectory(trajectoryName);
        // RamseteCommandDebug ramseteCommand = new RamseteCommandDebug(
        RamseteController EEEE = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);
        EEEE.setEnabled(false);
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            m_robotDrive::getPose,
            EEEE,
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            // new PIDController(DriveConstants.kPDriveVel, 0, 0), new
            // PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(0, 0, 0), new PIDController(0, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive
        );
        
        var cmd = new SequentialCommandGroup(
            Commands.runOnce(() -> { 
                Pose2d init = trajectory.getInitialPose();
                m_robotDrive.resetOdometry(new Pose2d(init.getX(), init.getY(), new Rotation2d(0)));
                SmartDashboard.putString("Trajg", init.toString());
            }),
            ramseteCommand,
            Commands.runOnce(() -> m_robotDrive.tankDriveVolts(0, 0)),
            Commands.runOnce(() -> { 
                Pose2d init = trajectory.getInitialPose();
                // m_robotDrive.resetOdometry(init);
                m_robotDrive.resetOdometry(new Pose2d(init.getX(), init.getY(), new Rotation2d(0)));

                SmartDashboard.putString("Trajg2", init.toString());
            })                );
        cmd.addRequirements(m_robotDrive);
        cmd.setName("trajectory: " + trajectoryName);
        return cmd;
	}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Trajectory getTrajectory(String trajectoryName) {
        // // String trajectoryJSON = "output/haha.wpilib.json";
        // // String trajectoryJSON =
        // // "pathplanner/generatedJSON/compact_backup_test.wpilib.json";
        // // String trajectoryJSON =
        // // "pathplanner/generatedJSON/long_turnless.wpilib.json";
        // // String trajectoryJSON =
        // // "pathplanner/generatedJSON/big_wide_turns.wpilib.json";
        // String trajectoryJSON = "pathplanner/generatedJSON/no_backup.wpilib.json";

        // Trajectory trajectory = new Trajectory();
        // try {
        // Path trajectoryPath =
        // Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        // trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // System.out.println("v\nv\nv\nv\ntrajectory total time" +
        // String.valueOf(trajectory.getTotalTimeSeconds()));
        // } catch (IOException ex) {
        // System.out.println("SHEEEEEESH");
        // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
        // ex.getStackTrace());
        // }

        // return exampleTrajectory;
        return trajectories.get(trajectoryName);
    }

    public Command balanceAuton(Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, BiConsumer<Double, Double> outputVolts) {
        var command = new SequentialCommandGroup(
            // 1. Position arm to place gamepiece
            // TODO: options: either place the game picee, or try to flip over, shoot, and
            // then come back so that we are in the same state

            // implement going over and shooting a cone?

            new ParallelCommandGroup(
                // 3. Retract arm
                // RobotMap.Component.arm.c_posReturnToHomeDown(false),
                new SequentialCommandGroup(
                    new WaitCommand(1), // TODO: set wait time to allow arm to get started before moving?
                    // 4. Drive forward past ramp
                    getAutonomousCommand("past_ramp"),

                    // 5. Drive back to get partially on ramp
                    getAutonomousCommand("back_to_ramp")
                )
            )
            // new Balance(RobotMap.Component.navx, wheelSpeeds, outputVolts, 1, -0.1)
            // 6. balance code here
        );

        return command;
    }

    public Command balanceAutonAndShootCube() {
        var cmd = RobotMap.Component.arm.c_shootCubes(4, () -> new SequentialCommandGroup(
            getAutonomousCommand("past_ramp"),
            getAutonomousCommand("back_to_ramp")
        )); // high shelf flippy

        return cmd;
    }

    public Command newAuton() {
        var cmd = RobotMap.Component.arm.c_shootCubes(4, () -> new SequentialCommandGroup(
            getAutonomousCommand("to_ramp"),
            getAutonomousCommand("angle_ramp_forward"),
            new WaitCommand(1),
            getAutonomousCommand("go_over_ramp"),
            getAutonomousCommand("angle_ramp_backward"),
            new WaitCommand(1),
            getAutonomousCommand("go_middle_ramp")
        ));

        return cmd;
    }
















    public final double ARM_PIVOT_SPEED = 180;  // 160 in teleop
    public final double ARM_PIVOT_ACCEL = 230;  // 210 in teleop

    // all auton movements assume retracted arm. use shootCones w/ autostow to ensure arm ends up retracted 
    public final BiFunction<Integer, Supplier<Command>, Command> autonPivotConeFlippy = (shelf, onArrivalCommandDealer) -> {
        var degreesFromHorizontal = ArmSubsystem.floorCones.get(shelf).getFirst();
        return new TriggerCommandFactory(() -> RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(degreesFromHorizontal, ARM_PIVOT_SPEED, ARM_PIVOT_ACCEL, onArrivalCommandDealer));
    };
    public final BiFunction<Integer, Supplier<Command>, Command> autonPivotCubeFlippy = (shelf, onArrivalCommandDealer) -> {
        var degreesFromHorizontal = ArmSubsystem.cubes.get(shelf).getFirst();
        return new TriggerCommandFactory(() -> RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(degreesFromHorizontal, ARM_PIVOT_SPEED, ARM_PIVOT_ACCEL, onArrivalCommandDealer));
    };

    public final BiFunction<Integer, Supplier<Command>, Command> autonShootCube = (shelf, onArrivalCommandDealer) -> {
        // assumes we're already at the desired angle
        var voltage = ArmSubsystem.cubes.get(shelf + 3).getThird();
        return new TriggerCommandFactory(() -> new SequentialCommandGroup(
            RobotMap.Component.intake.c_holdVoltage(voltage).withTimeout(0.5)
            , RobotMap.Component.intake.c_neutralOutput()
            , new TriggerCommandFactory(onArrivalCommandDealer)
        ));
    };
    public final BiFunction<Integer, Supplier<Command>, Command> autonShootCone = (shelf, onArrivalCommandDealer) -> new TriggerCommandFactory(
            // holdArmPose, shoot, then retract (but does not pivot to save time)
            () -> {
                var degreesFromHorizontal = ArmSubsystem.floorCones.get(shelf+3).getFirst();
                var extensionLengthMeters = ArmSubsystem.floorCones.get(shelf+3).getSecond();
                var voltage = ArmSubsystem.floorCones.get(shelf+3).getThird();

                return new TriggerCommandFactory(() -> RobotMap.Component.arm.c_holdArmPose(degreesFromHorizontal, extensionLengthMeters,
                    () -> nameCommand("auton shoot cone", new SequentialCommandGroup(
                        RobotMap.Component.intake.c_holdVoltage(voltage).withTimeout(0.5)
                                               .andThen(RobotMap.Component.intake.c_neutralOutput()),
                        new TriggerCommandFactory(() -> RobotMap.Component.arm.armExtensionSubsystem.c_holdExtension(0, 2, 3, onArrivalCommandDealer))
                    ) 
                )));
            });


    public final Supplier<Command> posAA_TO_AB_getPiece1 = () -> new SequentialCommandGroup(
        new ParallelDeadlineGroup(  // go to pickup location, while pivoting arm down and running intake
            getAutonomousCommand("go_to_pickup_next")
            , new TriggerCommandFactory(() -> RobotMap.Component.arm.c_posIntakeFloor(() -> RobotMap.Component.intake.c_holdVoltage(-8)))
        ),
        new ParallelDeadlineGroup(  // then return to the placement location while pivoting arm back up and holding rotation
            getAutonomousCommand("from_pickup_to_place")
            , autonPivotCubeFlippy.apply(3, null)
            , new TriggerCommandFactory(() -> RobotMap.Component.intake.c_holdItem())
        )
    );
    public final Supplier<Command> posAB_TO_BALANCE = () -> nameCommand("AB to Balance", new SequentialCommandGroup(
        getAutonomousCommand("from_cube_place_to_ramp_edge")
        , new WaitCommand(0.7)  //wait for ramp to lower,  TODO: needs tuning -- lower it as much as you can
        , getAutonomousCommand("onto_ramp")
    ));

    public Command twoPieceAuton() { // shoot cone, grab cube, shoot cube

        var afterConeShot = new SequentialCommandGroup(
            posAA_TO_AB_getPiece1.get(),
            autonShootCube.apply(3, RobotMap.Component.arm::c_posReturnToHomeUp)
            );
        var coneShot = autonShootCone.apply(3, null);

        var total_parallel = new ParallelCommandGroup(
            coneShot,
            (new WaitCommand(5).andThen(afterConeShot))
        );

        return total_parallel;
    }
    public Command twoPieceBalanceAuton() { // shoot cone, grab cube, shoot cube, doesn't balance
        // var cmd = autonShootCone.apply(3, 
        //     () -> new SequentialCommandGroup(
        //         posAA_TO_AB_getPiece1.get(),
        //         autonShootCube.apply(3,
        //             posAB_TO_BALANCE
        //         )
        //     )
        // );
        // SmartDashboard.putString("reqs", cmd.getRequirements().stream().map(s -> s.toString().split("\\.")).map(arr -> arr[arr.length-1]).collect(Collectors.joining(", ")));
        // return cmd;

        var afterConeShot = new SequentialCommandGroup(
            posAA_TO_AB_getPiece1.get(),
            autonShootCube.apply(3,
                posAB_TO_BALANCE
            ));
        var coneShot = autonShootCone.apply(3, null);
        var total_parallel = new ParallelCommandGroup(
            coneShot,
            (new WaitCommand(5).andThen(afterConeShot))
        );

        return total_parallel;
    }

    public Command hallwayPracticeAuton() { // shoot cone, grab cube, shoot cube, doesn't balance
        var onArrivalCommand = new BasedSequential(
            new BasedDeadline( // then, in parallel
                (new WaitCommand(1)).andThen(getAutonomousCommand("straight_forward")) // go to pickup location
                , new TriggerCommandFactory(() -> RobotMap.Component.arm.c_posIntakeFloor(() -> RobotMap.Component.intake.c_holdVoltage(-8.)))// start intaking when we get close
            ),
            new BasedDeadline( // then, as a separated parallel schedule,
                getAutonomousCommand("straight_backward") // return to the next placement location
                , new TriggerCommandFactory(() -> RobotMap.Component.intake.c_holdItem()) // hold the game piece in
            ),
            new TriggerCommandFactory(() -> RobotMap.Component.arm.c_shootCubes(5, () -> getAutonomousCommand("straight_forward"))) // finally, shoot the cube we just picked up and stow
        );

        var shootFirstCube = RobotMap.Component.arm.c_shootCubes(4);

        var total_parallel = new ParallelCommandGroup(
            new TriggerCommandFactory(() -> shootFirstCube),
            new TriggerCommandFactory(() -> (new WaitCommand(2.5)).andThen(onArrivalCommand))
        );
        return total_parallel;
    }
}
