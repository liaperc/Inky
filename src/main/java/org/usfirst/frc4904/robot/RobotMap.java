package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc4904.robot.subsystems.Intake;
import org.usfirst.frc4904.robot.subsystems.arm.ArmExtensionSubsystem;
import org.usfirst.frc4904.robot.subsystems.arm.ArmPivotSubsystem;
import org.usfirst.frc4904.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// import org.usfirst.frc4904.standard.LogKitten;

import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.chassis.SwerveDrive;
import org.usfirst.frc4904.standard.subsystems.chassis.SwerveModule;
import org.usfirst.frc4904.standard.subsystems.chassis.WestCoastDrive;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotMap {

    public static class Port {
        // public static class Network {
        //     public static SocketAddress LOCAL_SOCKET_ADDRESS = new InetSocketAddress(3375);
        //     public static SocketAddress LOCALIZATION_ADDRESS = new InetSocketAddress("10.49.04.10", 4321);
        // }

        public static class HumanInput {
            public static final int joystick = 0;
            public static final int xboxController = 1;
        }

        // // blinky constants


        
        // 2023 robot constants
        public static class CANMotor {
            public static final int RIGHT_DRIVE_A = 3;
            public static final int RIGHT_DRIVE_B = 4;
            public static final int LEFT_DRIVE_A = 1;
            public static final int LEFT_DRIVE_B = 2;

            public static final int PIVOT_MOTOR_LEFT = 11;
            public static final int PIVOT_MOTOR_RIGHT = 12;
            public static final int ARM_EXTENSION_MOTOR = 14;

            public static final int LEFT_INTAKE = 21;
            public static final int RIGHT_INTAKE = 22;
        }

        public static class PWM {
        }

        public static class CAN {
        }

        public static class Pneumatics {
        }

        public static class Digital {
        }

       }

    public static class Metrics {
        // blinky constants
        // public static class Chassis {
        //     public static final double GEAR_RATIO = 69/5;
        //     public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        //     public static final double TRACK_WIDTH_METERS = 0.59;
        // }

        // // 2023-robot constants
        public static class Chassis {
            public static final double GEAR_RATIO = 496/45; // https://www.desmos.com/calculator/llz7giggcf
            public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(5.12);
            public static final double TRACK_WIDTH_METERS = .8713; // +/- 0.5 inches
            public static final double TRACK_LENGTH_METERS = 1; //TODO: measure this 
            public static final double CHASSIS_LENGTH = Units.inchesToMeters(37); // +/- 0.5 inches
            public static final Translation2d CENTER_MASS_OFFSET = new Translation2d(0,0); // no offset
            public static final double EncoderTicksPerRevolution = 2048;

            //TODO: set 2024 swerve module metrics
            public static final double MAX_SPEED = -1;
            public static final double MAX_ACCELERATION = -1;
            public static final double MAX_TRANSLATION_SPEED = -1;
            public static final double MAX_TURN_SPEED = -1;
            public static final double MAX_TURN_ACCELERATION = -1;
            public static final double DRIVE_GEAR_RATIO = -1;
            public static final double TURN_GEAR_RATIO = -1;
        }
        

    }

    public static class PID {
        public static class Drive {
            // PID constants
            // public static final double kP = 1.5;
            public static final double kP = 3.3016;
            public static final double kI = 0;  // FIXME: tune
            public static final double kD = 0;
            // feedforward constants
            // pre-sfr on-carpet characterization
            // public static final double kS = 0.025236; 
            // public static final double kV = 3.0683;
            // public static final double kA = 0.7358;
            //post sfr characterization
            public static final double kS = 0.12507; 
            public static final double kV = 2.9669;
            public static final double kA = 0.67699;
        }

        public static class Turn { //TODO: tune
            // PID constants
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
            // feedforward constants
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
        }
    }

    public static class Component {
        // expose these for robotcontainer2
        //currently just frontleft and backright, will add mroe when we have more modules
        public static CANTalonFX FLdrive;
        public static CANTalonFX FLturn;
        public static CANTalonFX FRdrive;
        public static CANTalonFX FRturn;
        public static CANTalonFX BLdrive;
        public static CANTalonFX BLturn;
        public static CANTalonFX BRdrive;
        public static CANTalonFX BRturn;

        //encoders are dutycycle encoders, not standard can encoders
        public static DutyCycleEncoder FLturnEncoder;
        public static DutyCycleEncoder FRturnEncoder;
        public static DutyCycleEncoder BLturnEncoder;
        public static DutyCycleEncoder BRturnEncoder;

        public static AHRS navx;

        // public static RobotUDP robotUDP;

        public static SwerveDrive chassis;
        public static ArmSubsystem arm;
        public static Intake intake;
    }

    public static class NetworkTables {
        public static NetworkTableInstance instance;

        public static class Odometry {
            public static NetworkTable table;
            public static NetworkTableEntry pose;
            public static NetworkTableEntry accel;
            public static NetworkTableEntry turretAngle;
        }

        public static class Localization {
            public static NetworkTable table;
            public static NetworkTableEntry goalDistance;
            public static NetworkTableEntry goalRelativeAngle;
        }
    }

    public static class Input {
    }

    public static class HumanInput {
        public static class Driver {
            public static CustomCommandXbox xbox;
            public static CustomCommandJoystick xyJoystick;
            public static CustomCommandJoystick turnJoystick;
        }

        public static class Operator {
            public static CustomCommandJoystick joystick;
        }
    }

    public RobotMap() {
        Component.navx = new AHRS(SerialPort.Port.kMXP);

        HumanInput.Driver.xbox = new CustomCommandXbox(Port.HumanInput.xboxController, 0.01);
		HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick, 0.01);
        // // UDP things
        // try {
        //     Component.robotUDP = new RobotUDP(Port.Network.LOCAL_SOCKET_ADDRESS, Port.Network.LOCALIZATION_ADDRESS);
        // } catch (IOException ex) {
        //     LogKitten.f("Failed to initialize UDP subsystem");
        //     LogKitten.ex(ex);
        // }


        /***********************
         * Chassis Subsystem
        *************************/

        Component.FLdrive  = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_A, InvertType.None);
        Component.FLturn = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_B, InvertType.None);
        Component.BRdrive   = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_A, InvertType.None);
        Component.BRturn  = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_B, InvertType.None);


        // Component.backRightWheelTalon.setSafetyEnabled(false);
        // Component.frontRightWheelTalon.setSafetyEnabled(false);
        // Component.backLeftWheelTalon.setSafetyEnabled(false);
        // Component.frontLeftWheelTalon.setSafetyEnabled(false);

        //TalonMotorSubsystem rightDriveMotors = new TalonMotorSubsystem("right drive motors", NeutralMode.Brake, 0, Component.frontRightWheelTalon, Component.backRightWheelTalon);
        Translation2d locationFL = new Translation2d(Metrics.Chassis.TRACK_LENGTH_METERS / 2, Metrics.Chassis.TRACK_WIDTH_METERS / 2);
        Translation2d locationFR = new Translation2d(Metrics.Chassis.TRACK_LENGTH_METERS / 2, -(Metrics.Chassis.TRACK_WIDTH_METERS / 2));
        Translation2d locationBL = new Translation2d(-(Metrics.Chassis.TRACK_LENGTH_METERS / 2), Metrics.Chassis.TRACK_WIDTH_METERS / 2);
        Translation2d locationBR = new Translation2d(-(Metrics.Chassis.TRACK_LENGTH_METERS / 2), -(Metrics.Chassis.TRACK_WIDTH_METERS / 2));
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);

        SwerveModule FLmodule  = new SwerveModule(Component.FLdrive, Component.FLturn, Component.FLturnEncoder, locationFL);
        SwerveModule FRmodule = new SwerveModule(Component.FRdrive, Component.FRturn, Component.FRturnEncoder, locationFR);
        SwerveModule BLmodule   = new SwerveModule(Component.BLdrive, Component.BLturn, Component.BLturnEncoder, locationBL);
        SwerveModule BRmodule  = new SwerveModule(Component.BRdrive, Component.BRturn, Component.BRturnEncoder, locationBR);
        SwerveModule[] modules = {FLmodule, FRmodule, BLmodule, BRmodule};


        //SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getHeading()));
        Component.chassis = new SwerveDrive(modules, kinematics, Component.navx, Metrics.Chassis.CENTER_MASS_OFFSET, new Pose2d(0,0,new Rotation2d(0)));



        /***********************
         * Arm Subsystem
        *************************/

        CANTalonFX leftPivotMotor  = new CANTalonFX(RobotMap.Port.CANMotor.PIVOT_MOTOR_LEFT,  InvertType.InvertMotorOutput);
        CANTalonFX rightPivotMotor = new CANTalonFX(RobotMap.Port.CANMotor.PIVOT_MOTOR_RIGHT, InvertType.None);
        CANTalonFX armExtensionMotor = new CANTalonFX(Port.CANMotor.ARM_EXTENSION_MOTOR, InvertType.None);

        TalonMotorSubsystem armRotationMotors = new TalonMotorSubsystem("Arm Pivot Subsystem", NeutralMode.Brake, 0, leftPivotMotor, rightPivotMotor);
        ArmExtensionSubsystem armExtensionSubsystem = new ArmExtensionSubsystem(
            new TalonMotorSubsystem("Arm Extension Subsystem", NeutralMode.Brake, 0, armExtensionMotor),
            () -> ArmPivotSubsystem.motorRevsToAngle(armRotationMotors.getSensorPositionRotations() * 0.911 - 6.3)
        );
        // Autonomous.autonCommand = Component.chassis.c_buildPathPlannerAuto(
        //     PID.Drive.kS, PID.Drive.kV, PID.Drive.kA,
        //     Autonomous.RAMSETE_B, Autonomous.RAMSETE_ZETA,
        //     Autonomous.AUTON_NAME, Autonomous.MAX_VEL, Autonomous.MAX_ACCEL,
        //     Autonomous.autonEventMap
        // );

        ArmPivotSubsystem armPivotSubsystem = new ArmPivotSubsystem(armRotationMotors, armExtensionSubsystem::getCurrentExtensionLength);

        Component.arm = new ArmSubsystem(armPivotSubsystem, armExtensionSubsystem);

        /***********************
         * Intake Subsystem
        *************************/
        
        SparkMaxMotorSubsystem intake_left  = new SparkMaxMotorSubsystem("intake", IdleMode.kCoast, 0, new CustomCANSparkMax(Port.CANMotor.LEFT_INTAKE, MotorType.kBrushless, false));
        SparkMaxMotorSubsystem intake_right = new SparkMaxMotorSubsystem("intake", IdleMode.kCoast, 0, new CustomCANSparkMax(Port.CANMotor.RIGHT_INTAKE, MotorType.kBrushless, true));

        Component.intake = new Intake(intake_left, intake_right);
                
        // links we'll need
        // - angles and distances for intake/outtake: https://docs.google.com/spreadsheets/d/1B7Ie4efOpuZb4UQsk8lHycGvi6BspnF74DUMLmiKGUM/edit?usp=sharing
        // - naive + scuffed ramsete tuning: https://docs.google.com/spreadsheets/d/1BIvwJ6MfLf9ByW9dcmagXFvm7HPaXY78Y4YB1L9TGPA/edit#gid=0
    }
}