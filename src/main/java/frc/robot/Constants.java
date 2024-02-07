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
    public static final double stickDeadband = 0.1;

    // All buttons for the driver 

    public static final class Buttons {
        // LED Control
        public static final int LED_ORANGE = 1;
        public static final int LED_YELLOW = 2;
        public static final int LED_PURPLE = 3;

        public static final int INTAKE_ON = 1;
        public static final int INTAKE_OFF = 2;
    }

    public static final class Elevator {
        public static final int FOLLOWER_MOTOR_ID = 0;
        public static final int LEAD_MOTOR_ID = 0;

        public static final double ELEVATOR_KP = 0.0;
        public static final double ELEVATOR_KI = 0.0;
        public static final double ELEVATOR_KD = 0.0;
    }

    public static final class Intake {
        public static final int DRIVE_MOTOR = 21;
        public static final int SWIVEL_MOTOR = 34;
        public static final int INTAKE_SENSOR_ID = 0;

        public static final double SPEED = 0.5;

        public static final double SWIVEL_KP = 0.04;
        public static final double SWIVEL_KI = 0;
        public static final double SWIVEL_KD = 0;
        public static final double SWIVEL_KV = 0.058252;
        public static final double MOTMAGMAXACCEL = 34130 * 3; //6827
        public static final double MOTMAGMAXVEL = 34130 * 6; //34133
        
        public static final double DOWN_POSITION = 16.6; 
        public static final double UP_POSITION = 0;
    }

    public static final class Shooter {
        public static final int LEFT_MOTOR_ID = 40;
        public static final int RIGHT_MOTOR_ID = 30;

        public static double SHOOTER_KP = 0.0005;
        public static double SHOOTER_KF = 0.00011;
        public static double SHOOTER_KI = 0.00000025;
        public static double SHOOTER_KD = 0.0001;
    }

    // Lights Constants (Used in Lights.java)

    public final class Lights {
        // Lights states (console commands)
        public static final String LIGHT_STATE_OFF =  "{'on':false}";
        public static final String LIGHT_STATE_ON =  "{'on':true}";
        public static final String LIGHT_STATES_CHANGE_COLOR_YELLOW= "{'seg':[{'col': [[255, 200, 59, 0], [0, 0, 0, 0], [0, 0, 0, 0]]}]}";
        public static final String LIGHT_STATES_CHANGE_COLOR_PURPLE= "{'seg':[{'col': [[255, 0, 255, 0], [0, 0, 0, 0], [0, 0, 0, 0]]}]}";
        public static final String LIGHT_STATES_CHANGE_COLOR_ORANGE= "{'seg':[{'col': [[255, 129, 10, 0],[0, 0, 0, 0], [0, 0, 0, 0]]}]}";
        public static final String DEFAULT_LIGHT_STATE = LIGHT_STATE_OFF;
        public static final Map<LightStates,String> HASHMAP_LIGHT_STATES;

        // States to be used to interface with Hashmap
        public enum LightStates {
            OFF,
            ON,
            YELLOW,
            PURPLE, 
            ORANGE
        }
        
        // Lights hashmap
        static {
            HASHMAP_LIGHT_STATES = new HashMap<>();
            HASHMAP_LIGHT_STATES.put(LightStates.OFF,LIGHT_STATE_OFF);
            HASHMAP_LIGHT_STATES.put(LightStates.ON, LIGHT_STATE_ON);
            HASHMAP_LIGHT_STATES.put(LightStates.ORANGE, LIGHT_STATES_CHANGE_COLOR_ORANGE);
            HASHMAP_LIGHT_STATES.put(LightStates.YELLOW, LIGHT_STATES_CHANGE_COLOR_YELLOW);
            HASHMAP_LIGHT_STATES.put(LightStates.PURPLE, LIGHT_STATES_CHANGE_COLOR_PURPLE);
        }
    }

    public static final class Swerve {
        public static final int pigeonID = 14;
 
        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(25); 
        public static final double wheelBase = Units.inchesToMeters(25);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
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

        /* Swerve Current Limiting */
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
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(84.46);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-28.91);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(71.27);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(112.148);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
