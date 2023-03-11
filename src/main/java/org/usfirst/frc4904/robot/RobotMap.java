package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMax.IdleMode;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonSRX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDrive;
import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;
import org.usfirst.frc4904.standard.custom.sensors.EncoderPair;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import edu.wpi.first.wpilibj.SerialPort;
import org.usfirst.frc4904.standard.custom.sensors.NavX;

import org.usfirst.frc4904.standard.subsystems.chassis.SensorDrive;
mport org.usfirst.frc4904.robot.subsystems.net.RobotUDP;

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
            public static final int LEFT_INTAKE = -1; //TODO: fix
            public static final int RIGHT_INTAKE = -1; //TODO: fix
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
            public static final double DIAMETER_METERS = Units.inchesToMeters(-1.0); // TODO: Check values
            public static final double CIRCUMFERENCE_METERS = Metrics.Chassis.DIAMETER_METERS * Math.PI;
            public static final double TICKS_PER_METER = Metrics.Encoders.TalonEncoders.TICKS_PER_REVOLUTION
                    / Metrics.Chassis.CIRCUMFERENCE_METERS;
            public static final double DISTANCE_FRONT_BACK = Units.inchesToMeters(-1.0); // TODO: DOUBLE CHECK DISTANCES
            public static final double DISTANCE_SIDE_SIDE = Units.inchesToMeters(-1.0); // The robot's a square
            public static final double METERS_PER_TICK = Metrics.Chassis.CIRCUMFERENCE_METERS
                    / Metrics.Encoders.TalonEncoders.TICKS_PER_REVOLUTION;
            public static final double TURN_CORRECTION = 0.0;
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
        public static org.usfirst.frc4904.robot.subsystems.Intake intake;
        public static Motor rightWheelA;
        public static Motor rightWheelB;
        public static Motor leftWheelA;
        public static Motor leftWheelB;
        public static SensorDrive sensorDrive;
        public static TankDrive chassis;
        public static CustomPIDController drivePID;
        public static NavX navx;
     
        public static RobotUDP robotUDP;
        public static Pose2d initialPose;

        public static CANTalonFX climberTalon;
        public static Motor climberMotor;
        public static Climber climber;
        public static Shooter shooter;

        public static CustomCANSparkMax leftMotor;
        public static CustomCANSparkMax rightMotor;

        public static SparkMaxMotorSubsystem Intake;
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
            public static CustomXbox xbox;
        }

        public static class Operator {
            public static CustomJoystick joystick;
        }
    }

    public RobotMap() {
        Component.navx = new NavX(SerialPort.Port.kMXP);

        HumanInput.Driver.xbox = new CustomXbox(Port.HumanInput.xboxController);
		HumanInput.Operator.joystick = new CustomJoystick(Port.HumanInput.joystick);
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
        CANTalonFX leftWheelATalon = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_A);
        CANTalonFX leftWheelBTalon = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_B);
        CANTalonFX rightWheelATalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_A);
        CANTalonFX rightWheelBTalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_B);
        //intake
        CustomCANSparkMax intake_left = new CustomCANSparkMax(Port.CANMotor.LEFT_INTAKE, null, false);
        CustomCANSparkMax intake_right = new CustomCANSparkMax(Port.CANMotor.RIGHT_INTAKE, null, true);
        SparkMaxMotorSubsystem intake_motors = new SparkMaxMotorSubsystem("intake", IdleMode.kCoast, 11, intake_left, intake_right);
        Component.intake = new Intake(intake_motors);
        // Wheels
//        Component.rightWheelA = new Motor("rightWheelA", false, rightWheelATalon);
//        Component.rightWheelB = new Motor("rightWheelB", false, rightWheelBTalon);
//        Component.leftWheelA = new Motor("leftWheelA", true, leftWheelATalon);
//        Component.leftWheelB = new Motor("leftWheelB", true, leftWheelBTalon);

        // Wheel Encoders
        Component.leftWheelTalonEncoder = new CANTalonEncoder("leftWheel", leftWheelATalon, true,
                Metrics.Chassis.METERS_PER_TICK);
        Component.rightWheelTalonEncoder = new CANTalonEncoder("rightWheel", rightWheelATalon, true,
                Metrics.Chassis.METERS_PER_TICK);
        Component.initialPose = new Pose2d(); // TODO double x, double y, rotation2d
        Component.sensorDrive = new SensorDrive(Component.chassis, Component.leftWheelTalonEncoder,
                Component.rightWheelTalonEncoder, Component.navx, Component.initialPose);

        Component.chassisTalonEncoders = new EncoderPair(Component.leftWheelTalonEncoder,
                Component.rightWheelTalonEncoder);

        Component.chassis = new TankDrive("2022-Chassis", Component.leftWheelA, Component.leftWheelB,
                Component.rightWheelA, Component.rightWheelB);//, Component.shifter);
        Component.chassis.setDefaultCommand(new ChassisMove(Component.chassis, new NathanGain()));

        // NetworkTables setup
    }
}
