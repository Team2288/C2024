package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.sensors.BeamBreakSensor;

public class Intake extends SubsystemBase {
    private TalonFX swivel, drive;
    final MotionMagicVoltage motMag;
    private CurrentLimitsConfigs swivelCurrentLimits, driveCurrentLimits;
    public boolean hasNote, isRunning;
    public BeamBreakSensor sensor; 
    private Lights lights;

    public Intake(Lights lights) {
        // Initialize motors, motor controllers, and motor settings
        drive = new TalonFX(Constants.Intake.DRIVE_MOTOR);
        swivel = new TalonFX(Constants.Intake.SWIVEL_MOTOR);
        motMag = new MotionMagicVoltage(0);
        motMag.Slot = 0; 

        // Set factory defaults
        swivel.getConfigurator().apply(new TalonFXConfiguration());

        // Create and implement current limiter in configs
        swivelCurrentLimits = new CurrentLimitsConfigs();
        swivelCurrentLimits.StatorCurrentLimit = 40; // amps
        swivelCurrentLimits.StatorCurrentLimitEnable = true;
        var swivelTalonFXConfigs = new TalonFXConfiguration();
        swivelTalonFXConfigs.CurrentLimits = swivelCurrentLimits;

        // Set motion control settings
        swivelTalonFXConfigs.Slot0.kV = Constants.Intake.SWIVEL_KV * 2048 / 1023;
        swivelTalonFXConfigs.Slot0.kP = Constants.Intake.SWIVEL_KP * 2048 / 1023; // per new phoenix 6 units
        swivelTalonFXConfigs.Slot0.kI = Constants.Intake.SWIVEL_KI * 2048 / 1023 * 1000;
        swivelTalonFXConfigs.Slot0.kD = Constants.Intake.SWIVEL_KD * 2048 / 1023 / 1000;

        swivelTalonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.MOTMAGMAXVEL / 2048 * 10; // rps cruise velocity
        swivelTalonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Intake.MOTMAGMAXACCEL / 2048 * 10; // rps/s acceleration 
        swivelTalonFXConfigs.MotionMagic.MotionMagicJerk = 5000; // rps/s^2 jerk 
        
        swivel.getConfigurator().apply(swivelTalonFXConfigs, 0.050);
        
        driveCurrentLimits = new CurrentLimitsConfigs();
        driveCurrentLimits.StatorCurrentLimit = 40;
        driveCurrentLimits.StatorCurrentLimitEnable = true;
        var driveTalonFXConfigs = new TalonFXConfiguration();
        driveTalonFXConfigs.CurrentLimits = driveCurrentLimits;

        drive.getConfigurator().apply(driveTalonFXConfigs, 0.050);

        // Initialize beam break sensor
        sensor = new BeamBreakSensor(0);
        // Initialize lights
        this.lights = lights;
        // Initialize subsystem states
        hasNote = false;
        isRunning = false;
    }
    // Returns true if the beam is broken (something is in the intake)
    public boolean getSensor() {
        return sensor.getNoteDetected();
    }

    // Get roller speed
    public double getVelocity() {
        return drive.getVelocity().getValueAsDouble();
    }
    // Set roller speed    
    public void setDriveIntakeSpeed(double speed) {
        if (speed > 0.01) {this.isRunning = true;} else {this.isRunning = false;}
        drive.set(speed); 
    }

    // Get swivel position in rotations
    public double getPosition() {
        swivel.getRotorPosition().refresh();
        return swivel.getRotorPosition().getValueAsDouble();
    }
    // Set swivel position (in rotations)
    public void setPosition(double position) {
        swivel.setControl(motMag.withPosition(position));
    }

    // Stop swivel and rollers
    public void stopEverything() {
        System.out.println("Intake PosAndRun Command finished");
        setDriveIntakeSpeed(0.0);
        this.isRunning = false;
    }

    // Check if swivel is finished moving
    public boolean isCommandDone(double position) {
        if ((Math.abs(getPosition() - position) < 4) && sensor.getNoteDetected()) {
            return true;
        } else {
            return false;
        }
    }
    public boolean isCommandDonePositionOnly(double position) {
        return Math.abs(getPosition() - position) < 8;
    }

    // Move swivel and run rollers at *speed*
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
            new WaitCommand(.42)
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

    // Intake routines
    public Command getIntakeRoutineCommand() {
        return new SequentialCommandGroup(
            getPosAndRunIntakeCommand(Constants.Intake.DOWN_POSITION, Constants.Intake.SPEED),
            new ParallelCommandGroup(
                new InstantCommand(() -> lights.setState(Constants.Lights.ORANGE), lights),
                getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0)
            )
        );
    }
    public Command getAutoIntakeRoutineCommand() {
        return new SequentialCommandGroup(
            getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0),
            getPosAndRunIntakeCommand(Constants.Intake.DOWN_POSITION, Constants.Intake.SPEED),
            getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0)
        );
    }

    @Override
    public void periodic() {
        this.sensor.getNoteDetected();
        SmartDashboard.putBoolean("Sensor", sensor.getNoteDetected());
        SmartDashboard.putNumber("Swivel Encoder", getPosition());
        SmartDashboard.putNumber("Intake Velocity", getVelocity());
    }
}