package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    /* Deadband on the joystick */
    public static final double moveDeadband = 0.05;
    public static final double rotateDeadband = 0.05;

    /* Climber Constants */
    public static final class Climber {
        /* Climber CAN IDs */
        public static final int MOTOR_ID = 20;

        /* Motion Magic */
        public static final int MOTMAGMAXACCEL = 20480 * 16;
        public static final int MOTMAGMAXVEL = 2048 * 16;

        /* Climber PID Values No Load */
        public static final double kV = 0.0;
        public static final double kP = 0.06;
        public static final double kI = 0.000;
        public static final double kD = 0;
        public static final double kF = 0.0;

        /* Climber positions */
        public static final int UP_POSITION = -125;
        public static final int DOWN_POSITION = 143;
    }

    /* Elevator Constants */
    public static final class Elevator {
        /* Elevator CAN IDs */
        public static final int DRIVE_MOTOR_ID = 17;
        public static final int POSITION_MOTOR_ID = 18;
       // public static final int SENSOR_ID = 0;
    
        /* Elevator PID values */
        public static final int MOTMAGMAXACCELUP = 1000;
        public static final int MOTMAGMAXVELUP = 100;
        public static final double ELEVATOR_KP_UP = 0.057;
        public static final double ELEVATOR_KI_UP = 0.0014;
        public static final double ELEVATOR_KD_UP = 0.000;
        public static final double ELEVATOR_KV_UP = 0.117;

        public static final int MOTMAGMAXACCELDOWN = 500;
        public static final int MOTMAGMAXVELDOWN = 50;
        public static final double ELEVATOR_KP_DOWN = 0.053;
        public static final double ELEVATOR_KI_DOWN = 0.0014;
        public static final double ELEVATOR_KD_DOWN = 0.000;
        public static final double ELEVATOR_KV_DOWN = 0.115;

        /* Elevator Positions (rotations) */
        public static final int UP_AMP = 20; // 27 rotations
        public static final int UP_TRAP = 99;
        public static final int DOWN = 2;
    
        /* Elevator Speed */
        public static final double SPEED = 0.4;

        /* Trap flap positions (degrees based on pulse width) */
        public static final int RIGHT_FLAP_IN = 60;
        public static final int RIGHT_FLAP_OUT = 0;
        public static final int LEFT_FLAP_IN = 0;
        public static final int LEFT_FLAP_OUT = 60;
    }

    /* Intake Constants */
    public static final class Intake {
        /* Intake CAN IDs */
        public static final int DRIVE_MOTOR = 21;
        public static final int SWIVEL_MOTOR = 19;
        public static final int INTAKE_SENSOR_ID = 0;

        /* Intake PID Values */
        public static final double MOTMAGMAXACCEL = 34130 * 10; //6827
        public static final double MOTMAGMAXVEL = 34130 * 8; //34133        
        public static final double SWIVEL_KP = 0.11;
        public static final double SWIVEL_KI = 0;
        public static final double SWIVEL_KD = 0;
        public static final double SWIVEL_KV = 0.058252;

        /* Intake Positions */
        public static final double DOWN_POSITION = 17.8; 
        public static final double UP_POSITION = 0;

        /* Intake Speed */
        public static final double SPEED = 0.8; //.25    
    }

    /* Shooter Constants */
    public static final class Shooter {
        /* Shooter CAN IDs */
        public static final int LEFT_MOTOR_ID = 16;
        public static final int RIGHT_MOTOR_ID =15; 

        /* Shooter PID Values */
        public static double SHOOTER_KP = 0.0004;
        public static double SHOOTER_KF = 0.00019;
        public static double SHOOTER_KI = 0.000000;
        public static double SHOOTER_KD = 0.000;
    }

    /* Lights Constants (Used in Lights.java) */
    public final class Lights {
        /* Light states (console commands) */
        public static final double LIGHTS_OFF = 0.0;
        public static final double PURPLE = 0.90;
        public static final double ORANGE = 0.62;
        public static final double GREEN = 0.75; 
        public static final double DEFAULT_LIGHT_STATE = PURPLE;
        //public static final Map<LightStates,Double> HASHMAP_LIGHT_STATES;

        /* States to be used to interface with Hashmap *
        public enum LightStates {
            OFF,
            PURPLE, 
            ORANGE,
            GREEN
        }
        
        /* Lights hashmap *
        static {
            HASHMAP_LIGHT_STATES = new HashMap<>();
            HASHMAP_LIGHT_STATES.put(LightStates.OFF, LIGHT_STATE_OFF);
            HASHMAP_LIGHT_STATES.put(LightStates.PURPLE, LIGHT_STATES_CHANGE_COLOR_PURPLE);
            HASHMAP_LIGHT_STATES.put(LightStates.ORANGE, LIGHT_STATES_CHANGE_COLOR_ORANGE);
            HASHMAP_LIGHT_STATES.put(LightStates.GREEN, LIGHT_STATES_CHANGE_COLOR_GREEN);
        }
        */
    }

    /* Swerve Constants */
    public static final class Swerve {
        public static final int pigeonID = 14;
 
        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(25); 
        public static final double wheelBase = Units.inchesToMeters(25);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics */
            // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Current Limiters */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // 0.12? 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.14163; // Divide by 12
        public static final double driveKV = 2.2535;
        public static final double driveKA = 0.98001;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
            // ^ Meters per second
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot
            // ^ radians per second

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        // Front Left Module - Module 0
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(84.46);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // Front Right Module - Module 1
        public static final class Mod1 { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-28.91);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        // Back Left Module - Module 2
        public static final class Mod2 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(71.27);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // Back Right Module - Module 3
        public static final class Mod3 { 
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(112.148);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    /* Autonomous Constants */
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 7.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 10;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 10;
    
        public static final double kPXController = 1.2;
        public static final double kPYController = 1.2;
        public static final double kPThetaController = 13.2;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}