package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;

import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.custom.motorcontrollers.TalonMotorController;
import org.usfirst.frc4904.standard.subsystems.chassis.WestCoastDrive;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.standard.custom.sensors.EncoderPair;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.usfirst.frc4904.standard.custom.sensors.NavX;

import org.usfirst.frc4904.robot.subsystems.net.RobotUDP;

public class RobotMap {
    public static class Port {
        public static class Network {
            public static SocketAddress LOCAL_SOCKET_ADDRESS = new InetSocketAddress(3375);
            public static SocketAddress LOCALIZATION_ADDRESS = new InetSocketAddress("10.49.04.10", 4321);
        }

        public static class HumanInput {
            public static final int joystick = 0;
            public static final int xboxController = 1;
        }

        public static class CANMotor {
            public static final int RIGHT_DRIVE_A = 3; // TODO: Check chassis motor IDs
            public static final int RIGHT_DRIVE_B = 4;
            public static final int LEFT_DRIVE_A = 1;
            public static final int LEFT_DRIVE_B = 2;
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
        public static class Chassis {
            public static final double GEAR_RATIO = 5.0; // TODO: Check gear ratio for robot
            public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(-1.0); // TODO: Check values
            public static final double WHEEL_CIRCUMFERENCE_METERS = Metrics.Chassis.WHEEL_DIAMETER_METERS * Math.PI;
            public static final double TICKS_PER_METER = Metrics.Encoders.TalonEncoders.TICKS_PER_REVOLUTION
                    / Metrics.Chassis.WHEEL_CIRCUMFERENCE_METERS;
            public static final double DISTANCE_FRONT_BACK = Units.inchesToMeters(-1.0); // TODO: DOUBLE CHECK DISTANCES
            public static final double DISTANCE_SIDE_SIDE = Units.inchesToMeters(-1.0); // The robot's a square
            public static final double METERS_PER_TICK = Metrics.Chassis.WHEEL_CIRCUMFERENCE_METERS
                    / Metrics.Encoders.TalonEncoders.TICKS_PER_REVOLUTION / Metrics.Chassis.GEAR_RATIO;
            public static final double TURN_CORRECTION = 0.0;
            public static final double TRACK_WIDTH_METERS = 0; //TODO: change to actual value
        }

        public static class Encoders {
            public static class TalonEncoders {
                public static final double TICKS_PER_REVOLUTION = 2048.0;
                public static final double REVOLUTIONS_PER_TICK = 1 / TICKS_PER_REVOLUTION;
            }
        }
    }

    public static class PID {
        public static class Drive {
        }

        public static class Turn {
        }
    }

    public static class Component {
        public static CANTalonEncoder leftWheelTalonEncoder;
        public static CANTalonEncoder rightWheelTalonEncoder;
        public static EncoderPair chassisTalonEncoders;
        
        public static NavX navx;
     
        public static RobotUDP robotUDP;
        public static Pose2d initialPose;

        public static TalonMotorSubsystem leftDriveMotors;
        public static TalonMotorSubsystem rightDriveMotors;
        public static WestCoastDrive<TalonMotorController> Chassis;
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
            public static CommandXboxController xbox;
        }

        public static class Operator {
            public static CustomCommandJoystick joystick;
        }
    }

    public RobotMap() {
        Component.navx = new NavX(SerialPort.Port.kMXP);

        HumanInput.Driver.xbox = new CommandXboxController(Port.HumanInput.xboxController);
		HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick);
        // UDP things
        try {
            Component.robotUDP = new RobotUDP(Port.Network.LOCAL_SOCKET_ADDRESS, Port.Network.LOCALIZATION_ADDRESS);
        } catch (IOException ex) {
            LogKitten.f("Failed to initialize UDP subsystem");
            LogKitten.ex(ex);
        }

        // Chassis

        /* Drive Train */
        // TalonFX
        CANTalonFX rightWheelATalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_A, InvertType.None);
        CANTalonFX rightWheelBTalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_B, InvertType.FollowMaster);
        CANTalonFX leftWheelATalon  = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_A, InvertType.InvertMotorOutput);
        CANTalonFX leftWheelBTalon  = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_B, InvertType.FollowMaster);

        // components
        Component.leftDriveMotors  = new TalonMotorSubsystem("left drive motors",  NeutralMode.Brake, 10,  leftWheelATalon,  leftWheelBTalon);
        Component.rightDriveMotors = new TalonMotorSubsystem("right drive motors", NeutralMode.Brake, 10, rightWheelATalon, rightWheelBTalon);
        Component.Chassis = new WestCoastDrive<TalonMotorController>(Metrics.Chassis.TRACK_WIDTH_METERS, Metrics.Chassis.GEAR_RATIO, Metrics.Chassis.WHEEL_DIAMETER_METERS, Component.leftDriveMotors, Component.rightDriveMotors);


        // Wheel Encoders -- UNUSED
        
        // NetworkTables setup
    }
}
