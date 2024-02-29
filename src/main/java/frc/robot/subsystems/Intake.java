package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.robot.sensors.TOFSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class Intake extends SubsystemBase {
    CANSparkMax driveNeo; // motors
    TalonFX swivelFalcon;
    boolean hasNote, isRunning;
    public TOFSensor sensor; 
    final MotionMagicVoltage motMag;
    private CurrentLimitsConfigs currentLimits;

    public Intake() {
        driveNeo = new CANSparkMax(Constants.Intake.DRIVE_MOTOR, MotorType.kBrushless);
        driveNeo.setIdleMode(IdleMode.kBrake);
        driveNeo.setSmartCurrentLimit(40);

        swivelFalcon = new TalonFX(Constants.Intake.SWIVEL_MOTOR);
        currentLimits = new CurrentLimitsConfigs();

        currentLimits.StatorCurrentLimit = 40; // amps
        currentLimits.StatorCurrentLimitEnable = true;

        sensor = new TOFSensor(Constants.Intake.INTAKE_SENSOR_ID);

        motMag = new MotionMagicVoltage(0);
        motMag.Slot = 0;
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.CurrentLimits = currentLimits; // put current limiter in the configs

        swivelFalcon.getConfigurator().apply(new TalonFXConfiguration()); // set factory default

        talonFXConfigs.Slot0.kV = Constants.Intake.SWIVEL_KV * 2048 / 1023;

        talonFXConfigs.Slot0.kP = Constants.Intake.SWIVEL_KP * 2048 / 1023; // per new phoenix 6 units
        talonFXConfigs.Slot0.kI = Constants.Intake.SWIVEL_KI * 2048 / 1023 * 1000;
        talonFXConfigs.Slot0.kD = Constants.Intake.SWIVEL_KD * 2048 / 1023 / 1000;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.MOTMAGMAXVEL / 2048 * 10; // rps cruise velocity
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Intake.MOTMAGMAXACCEL / 2048 * 10; // rps/s acceleration 
        talonFXConfigs.MotionMagic.MotionMagicJerk = 3200; // rps/s^2 jerk 
        
        swivelFalcon.getConfigurator().apply(talonFXConfigs, 0.050);
        hasNote = false;
        isRunning = false;
    }

    public void setDriveIntakeSpeed(double speed) {
        if (speed > 0.01) {this.isRunning = true;} else {this.isRunning = false;}

        driveNeo.set(speed); 
    }

    public double getSensor() {
        return this.sensor.getRangeDebugger();
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
        return ((Math.abs(getPosition() - position) < 4) && this.sensor.getNoteDetected());
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
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
        return seqgroup.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    }


    public Command getIntakeDriveCommand(double speed) {
        return new FunctionalCommand(
            () -> {System.out.println("Starting Drive Cmd at speed: " + speed);},
            () -> setDriveIntakeSpeed(speed),
            interrupted -> {System.out.println("Ended drive cmd whose speed was: " + speed);},
            () -> (true),
            this
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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


    public Command getIntakeRoutineCommand() {
        return new SequentialCommandGroup(
            getPosAndRunIntakeCommand(Constants.Intake.DOWN_POSITION, Constants.Intake.SPEED),
            getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0)
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("TOF Sensor", this.sensor.getRangeDebugger());
        SmartDashboard.putNumber("Swivel Falcon Encoder", getPosition());
    }
}