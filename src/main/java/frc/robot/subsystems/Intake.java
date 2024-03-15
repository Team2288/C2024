package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.Constants.Lights.LightStates;
import frc.robot.sensors.TOFSensor;
import frc.robot.sensors.BeamBreakSensor;
import frc.robot.subsystems.Lights;

public class Intake extends SubsystemBase {
    CANSparkMax driveNeo; // motors
    TalonFX swivelFalcon;
    final MotionMagicVoltage motMag;
    private CurrentLimitsConfigs currentLimits;
    boolean hasNote, isRunning;
    public BeamBreakSensor sensor; 

    public Intake() {
        // Initialize motors, motor controllers, and motor settings
        driveNeo = new CANSparkMax(Constants.Intake.DRIVE_MOTOR, MotorType.kBrushless);
        driveNeo.setIdleMode(IdleMode.kBrake);
        driveNeo.setSmartCurrentLimit(40);

        swivelFalcon = new TalonFX(Constants.Intake.SWIVEL_MOTOR);
        motMag = new MotionMagicVoltage(0);
        motMag.Slot = 0; 

        // Set factory defaults
        swivelFalcon.getConfigurator().apply(new TalonFXConfiguration());

        // Create and implement current limiter in configs
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 40; // amps
        currentLimits.StatorCurrentLimitEnable = true;
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.CurrentLimits = currentLimits;

        // Set motion control settings
        talonFXConfigs.Slot0.kV = Constants.Intake.SWIVEL_KV * 2048 / 1023;
        talonFXConfigs.Slot0.kP = Constants.Intake.SWIVEL_KP * 2048 / 1023; // per new phoenix 6 units
        talonFXConfigs.Slot0.kI = Constants.Intake.SWIVEL_KI * 2048 / 1023 * 1000;
        talonFXConfigs.Slot0.kD = Constants.Intake.SWIVEL_KD * 2048 / 1023 / 1000;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.MOTMAGMAXVEL / 2048 * 10; // rps cruise velocity
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Intake.MOTMAGMAXACCEL / 2048 * 10; // rps/s acceleration 
        talonFXConfigs.MotionMagic.MotionMagicJerk = 3200; // rps/s^2 jerk 
        
        swivelFalcon.getConfigurator().apply(talonFXConfigs, 0.050);

        // Initialize time of flight sensor

        sensor = new BeamBreakSensor(0);
        // Initialize subsystem states
        hasNote = false;
        isRunning = false;
    }

    public void setDriveIntakeSpeed(double speed) {
        if (speed > 0.01) {this.isRunning = true;} else {this.isRunning = false;}
        driveNeo.set(speed); 
    }

    // Returns true if the beam is broken (something is in the intake)
    public boolean getSensor() {
        return sensor.getNoteDetected();
    }

    public void testSwivel(double speed) {
        swivelFalcon.set(speed);
    }

    public void setPosition(double position) {
        swivelFalcon.setControl(motMag.withPosition(position));
    }

    public double getPosition() { // returns rotations
        swivelFalcon.getRotorPosition().refresh();
        return swivelFalcon.getRotorPosition().getValueAsDouble();
    }

    public void stopEverything() {
        System.out.println("Intake PosAndRun Command finished");
        setDriveIntakeSpeed(0.0);
        this.isRunning = false;
    }

    public boolean isCommandDone(double position) {
        return ((Math.abs(getPosition() - position) < 4) && sensor.getNoteDetected());
    }

    public boolean isCommandDonePositionOnly(double position) {
        return Math.abs(getPosition() - position) < 8;
    }

    public Command getPosAndRunIntakeCommand(double position, double speed) {
        return new FunctionalCommand(
            () -> {
                System.out.println("Going to position:" + position + " At speed: " + speed); 
                this.setDriveIntakeSpeed(speed); 
            }, 
            () -> setPosition(position),
            interrupted -> stopEverything(),
            () -> isCommandDone(position),
            this
        );
    }

    public Command getShootCommandNoRamp() {
        SequentialCommandGroup seqgroup = new SequentialCommandGroup();

        seqgroup.addCommands(
            this.getIntakeDriveCommand(Constants.Intake.SPEED),
            new WaitCommand(.7)
        );

        /* 
        if (this.s_Intake.getPosition() < 4) {
            seqgroup.addCommands(this.s_Intake.getIntakeDriveCommand(0.0));
        } else {
            seqgroup.addCommands(
                this.s_Intake.getPositionCommand(Constants.Intake.UP_POSITION),
                this.s_Intake.getIntakeDriveCommand(0.0)
            );
        }
        */

        seqgroup.addCommands(this.getIntakeDriveCommand(0.0));

        seqgroup.addRequirements(this);
        return seqgroup;

    }

    public Command getIntakeDriveCommand(double speed) {
        return new FunctionalCommand(
            () -> {System.out.println("Starting Drive Cmd at speed: " + speed);},
            () -> setDriveIntakeSpeed(speed),
            interrupted -> {System.out.println("Ended drive cmd whose speed was: " + speed);},
            () -> (true),
            this
        );
    }

    public Command getIntakeDriveCommandNeedsNote(double speed) {
        return new FunctionalCommand(
            () -> {System.out.println("Starting drive cmd needs note");},
            () -> setDriveIntakeSpeed(speed),
            interrupted -> {System.out.println("Ended drive cmd needs note");},
            () -> (this.sensor.getNoteDetected()),
            this
        );
    }

    public Command getPositionCommand(double pos) {
        return new FunctionalCommand(
            () -> System.out.println("Running Position Command"),
            () -> setPosition(pos),
            interrupted -> {System.out.println("Done w/ position command");},
            () -> isCommandDonePositionOnly(pos),
            this
        );
    }

    public Command getAutoIntakeRoutineCommand() {
        return new SequentialCommandGroup(
            getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0),
            getPosAndRunIntakeCommand(Constants.Intake.DOWN_POSITION, Constants.Intake.SPEED),
            getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0)
        );
    }

    public Command getIntakeRoutineCommand() {
        return new SequentialCommandGroup(
            getPosAndRunIntakeCommand(Constants.Intake.DOWN_POSITION, Constants.Intake.SPEED),
            getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0)
        );
    }

    @Override
    public void periodic() {
        this.sensor.getNoteDetected();
        SmartDashboard.putBoolean("Sensor", sensor.getNoteDetected());
        SmartDashboard.putNumber("Swivel Falcon Encoder", getPosition());
    }
}