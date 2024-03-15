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
        public static final int MOTOR_ID = 30;

        /* Climber PID Values */
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        /* Climber positions */
        public static final int UP_POSITION = 0;
        public static final int DOWN_POSITION = -12 * 7 * 2048;
            // -12 (Gear ratio), 7 (rotations needed), 2048 (encoder ticks / rotation)
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
        public static final int MOTMAGMAXACCELDOWN = 200; //19240
        public static final int MOTMAGMAXVELDOWN = 20; //1924        
        public static final double ELEVATOR_KP = 0.051; //0.00909488172
        public static final double ELEVATOR_KI = 0.0014;
        public static final double ELEVATOR_KD = 0.000;
        public static final double ELEVATOR_KV = 0.117; //0.05011894428

        /* Elevator Positions (rotations) */
        public static final int UP_AMP = 40; // rotations - 68 max
        public static final int DOWN = 2;
    
        /* Elevator Speed */
        public static final double SPEED = 0.32;
    }

    /* Intake Constants */
    public static final class Intake {
        /* Intake CAN IDs */
        public static final int DRIVE_MOTOR = 33;
        public static final int SWIVEL_MOTOR = 34;
        public static final int INTAKE_SENSOR_ID = 0;

        /* Intake PID Values */
        public static final double MOTMAGMAXACCEL = 34130 * 3; //6827
        public static final double MOTMAGMAXVEL = 34130 * 6; //34133        
        public static final double SWIVEL_KP = 0.04;
        public static final double SWIVEL_KI = 0;
        public static final double SWIVEL_KD = 0;
        public static final double SWIVEL_KV = 0.058252;

        /* Intake Positions */
        public static final double DOWN_POSITION = 16.6; 
        public static final double UP_POSITION = 0;

        /* Intake Speed */
        public static final double SPEED = 0.8;        
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
        public static final String LIGHT_STATE_OFF = "{'on': false}";
        public static final String LIGHT_STATE_ON =  "{'on':true}";
        public static final String LIGHT_STATES_CHANGE_COLOR_YELLOW= "{'seg':[{'col': [[255, 200, 59, 0], [0, 0, 0, 0], [0, 0, 0, 0]]}]}";
        //public static final String LIGHT_STATES_CHANGE_COLOR_PURPLE= "{'seg':[{'col': [[255, 0, 255, 0], [0, 0, 0, 0], [0, 0, 0, 0]]}]}";
        public static final String LIGHT_STATES_CHANGE_COLOR_PURPLE = "{\"on\":true,\"bri\":8,\"transition\":2,\"mainseg\":3,\"seg\":[{\"id\":0,\"start\":0,\"stop\":4,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg1\",\"col\":[[255,0,255],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":1,\"start\":4,\"stop\":8,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg2\",\"col\":[[255,0,255],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":2,\"start\":8,\"stop\":12,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg3\",\"col\":[[255,0,255],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":3,\"start\":12,\"stop\":16,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":84,\"cct\":127,\"set\":0,\"n\":\"Seg4\",\"col\":[[255,0,255],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0}]}";
        //public static final String LIGHT_STATES_CHANGE_COLOR_ORANGE= "{'seg':[{'col': [[255, 129, 10, 0],[0, 0, 0, 0], [0, 0, 0, 0]]}]}";
        public static final String LIGHT_STATES_CHANGE_COLOR_ORANGE = "{\"on\":true,\"bri\":8,\"transition\":2,\"mainseg\":3,\"seg\":[{\"id\":0,\"start\":0,\"stop\":4,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg1\",\"col\":[[255,129,10],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":1,\"start\":4,\"stop\":8,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg2\",\"col\":[[255,129,10],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":2,\"start\":8,\"stop\":12,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg3\",\"col\":[[255,129,10],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":3,\"start\":12,\"stop\":16,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":84,\"cct\":127,\"set\":0,\"n\":\"Seg4\",\"col\":[[255,129,10],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0}]}";
        public static final String LIGHT_STATES_SEG1 = "{\"on\":true,\"bri\":8,\"transition\":2,\"mainseg\":3,\"seg\":[{\"id\":0,\"start\":0,\"stop\":4,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg1\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":1,\"start\":4,\"stop\":8,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg2\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":2,\"start\":8,\"stop\":12,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg3\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":3,\"start\":12,\"stop\":16,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":84,\"cct\":127,\"set\":0,\"n\":\"Seg4\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0}]}";
        public static final String LIGHT_STATES_SEG2 = "{\"on\":true,\"bri\":8,\"transition\":2,\"mainseg\":3,\"seg\":[{\"id\":0,\"start\":0,\"stop\":4,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg1\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":1,\"start\":4,\"stop\":8,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg2\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":2,\"start\":8,\"stop\":12,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg3\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":3,\"start\":12,\"stop\":16,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":84,\"cct\":127,\"set\":0,\"n\":\"Seg4\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0}]}";
        public static final String LIGHT_STATES_SEG3 = "{\"on\":true,\"bri\":8,\"transition\":2,\"mainseg\":3,\"seg\":[{\"id\":0,\"start\":0,\"stop\":4,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg1\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":1,\"start\":4,\"stop\":8,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg2\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":2,\"start\":8,\"stop\":12,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg3\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":3,\"start\":12,\"stop\":16,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":84,\"cct\":127,\"set\":0,\"n\":\"Seg4\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0}]}";
        public static final String LIGHT_STATES_SEG4 = "{\"on\":true,\"bri\":8,\"transition\":2,\"mainseg\":3,\"seg\":[{\"id\":0,\"start\":0,\"stop\":4,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg1\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":1,\"start\":4,\"stop\":8,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg2\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":2,\"start\":8,\"stop\":12,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":false,\"frz\":false,\"bri\":255,\"cct\":127,\"set\":0,\"n\":\"Seg3\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"id\":3,\"start\":12,\"stop\":16,\"startY\":0,\"stopY\":8,\"grp\":1,\"spc\":0,\"of\":0,\"on\":true,\"frz\":false,\"bri\":84,\"cct\":127,\"set\":0,\"n\":\"Seg4\",\"col\":[[47,255,0],[0,0,0],[0,0,0]],\"fx\":0,\"sx\":128,\"ix\":128,\"pal\":0,\"c1\":128,\"c2\":128,\"c3\":16,\"sel\":true,\"rev\":false,\"mi\":false,\"rY\":false,\"mY\":false,\"tp\":false,\"o1\":false,\"o2\":false,\"o3\":false,\"si\":0,\"m12\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0},{\"stop\":0}]}";
        public static final String DEFAULT_LIGHT_STATE = LIGHT_STATE_OFF;
        public static final Map<LightStates,String> HASHMAP_LIGHT_STATES;

        /* States to be used to interface with Hashmap */
        public enum LightStates {
            OFF,
            ON,
            YELLOW,
            PURPLE, 
            ORANGE,
            SEG1,
            SEG2,
            SEG3,
            SEG4
        }
        
        /* Lights hashmap */
        static {
            HASHMAP_LIGHT_STATES = new HashMap<>();
            HASHMAP_LIGHT_STATES.put(LightStates.OFF,LIGHT_STATE_OFF);
            HASHMAP_LIGHT_STATES.put(LightStates.ON, LIGHT_STATE_ON);
            HASHMAP_LIGHT_STATES.put(LightStates.ORANGE, LIGHT_STATES_CHANGE_COLOR_ORANGE);
            HASHMAP_LIGHT_STATES.put(LightStates.YELLOW, LIGHT_STATES_CHANGE_COLOR_YELLOW);
            HASHMAP_LIGHT_STATES.put(LightStates.PURPLE, LIGHT_STATES_CHANGE_COLOR_PURPLE);
            HASHMAP_LIGHT_STATES.put(LightStates.SEG1, LIGHT_STATES_SEG1);
            HASHMAP_LIGHT_STATES.put(LightStates.SEG2, LIGHT_STATES_SEG2);
            HASHMAP_LIGHT_STATES.put(LightStates.SEG3, LIGHT_STATES_SEG3);
            HASHMAP_LIGHT_STATES.put(LightStates.SEG4, LIGHT_STATES_SEG4);
        }
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
        public static final double kMaxAccelerationMetersPerSecondSquared = 9;
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