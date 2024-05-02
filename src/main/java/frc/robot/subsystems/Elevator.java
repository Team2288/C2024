package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.sensors.TOFSensor;

import java.util.ArrayList;

public class Elevator extends SubsystemBase {
    private TalonFX driveMotor, elevatorMotor;
    private CurrentLimitsConfigs currentConfigs;
    private TalonFXConfiguration talonFXConfigs;
    final MotionMagicVoltage motMag;
    public double kP, kI, kD, kF, kMaxOutput, kMinOutput;

    TOFSensor sensor;
    public ArrayList<String[]> loggingData;
    Servo rightFlap, leftFlap;

    public Elevator() {
        super();
        // Initialize motors, motor controllers, and settings
        driveMotor = new TalonFX(Constants.Elevator.DRIVE_MOTOR_ID);
        elevatorMotor = new TalonFX(Constants.Elevator.POSITION_MOTOR_ID);


        motMag = new MotionMagicVoltage(0);
        motMag.Slot = 0;

        // Set factory defaults
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());

        // Create and implement current limiter in configs
        talonFXConfigs = new TalonFXConfiguration();
        currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = 80;
        currentConfigs.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits = currentConfigs;

        // Set motion control settings
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        talonFXConfigs.Slot0.kV = Constants.Elevator.ELEVATOR_KV_UP;
        talonFXConfigs.Slot0.kP = Constants.Elevator.ELEVATOR_KP_UP; 
        talonFXConfigs.Slot0.kI = Constants.Elevator.ELEVATOR_KI_UP;
        talonFXConfigs.Slot0.kD = Constants.Elevator.ELEVATOR_KD_UP;
        talonFXConfigs.Slot1.kV = Constants.Elevator.ELEVATOR_KV_DOWN;
        talonFXConfigs.Slot1.kP = Constants.Elevator.ELEVATOR_KP_DOWN; 
        talonFXConfigs.Slot1.kI = Constants.Elevator.ELEVATOR_KI_DOWN;
        talonFXConfigs.Slot1.kD = Constants.Elevator.ELEVATOR_KD_DOWN;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MOTMAGMAXVELUP; // rps cruise velocity
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MOTMAGMAXACCELUP; // rps/s acceleration 
        talonFXConfigs.MotionMagic.MotionMagicJerk = 25000; // rps/s^2 jerk 
        
        elevatorMotor.getConfigurator().apply(talonFXConfigs, 0.050);

        //Initialize servos (trap flaps)
        rightFlap = new Servo(0);
        leftFlap = new Servo(1);

        // Initialize TOF sensor
        sensor = new TOFSensor(0);
    }

    public Command feedNoteTrap(double speed) {
        return new FunctionalCommand(
            () -> {System.out.println("Feeding note for trap");},
            () -> {setElevatorSpeed(speed);},
            interrupted -> {System.out.println("Ended trap note feed"); setElevatorSpeed(0.0);},
            () -> (this.sensor.getNoteDetected()),
            this
        );
    }

    public void setElevatorSpeed(double speed) {
        driveMotor.set(-speed);
    }

    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }
    public void setElevatorPosition(double rotations) {
        if(rotations <= 5) {
            talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MOTMAGMAXVELDOWN;
            talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MOTMAGMAXACCELDOWN;
            elevatorMotor.getConfigurator().apply(talonFXConfigs.Slot1, 0.050);
        } else {
            talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MOTMAGMAXVELUP;
            talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MOTMAGMAXACCELUP;
            elevatorMotor.getConfigurator().apply(talonFXConfigs.Slot0, 0.050);
        }
        elevatorMotor.setControl(motMag.withPosition(rotations));
    }
    public Command getElevatorPositionCommand(double rotations) {
        return new FunctionalCommand(
            () -> System.out.println("Running elevator"),
            () -> setElevatorPosition(rotations),
            interrupted -> {System.out.println("elevator interrupted");},
            () -> Math.abs(getPosition() - rotations) < 2,
            this
        );
    }

    public void setFlapAngles(double rightSideAngle, double leftSideAngle) {
        rightFlap.setAngle(rightSideAngle);
        leftFlap.setAngle(leftSideAngle);
    }
    public Command getFlapsCommand(double rightSideAngle, double leftSideAngle) {
        return new InstantCommand(() -> setFlapAngles(rightSideAngle, leftSideAngle), this);
    }
    
    public void SmartDashboard() {
        SmartDashboard.putNumber("Sensor Reading", sensor.getRangeDebugger());
        SmartDashboard.putNumber("Elevator Position", elevatorMotor.getPosition().getValue());
        SmartDashboard.putNumber("Elevator position graph", elevatorMotor.getPosition().getValue());
        SmartDashboard.putNumber("elevator Roller speed", driveMotor.getVelocity().getValue());
    }
    @Override
    public void periodic() {
        SmartDashboard();
    }
}