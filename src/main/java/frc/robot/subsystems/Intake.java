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
import frc.robot.TOFSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class Intake extends SubsystemBase {
    CANSparkMax driveNeo; // motors
    TalonFX swivelFalcon;
    boolean hasNote, isRunning;
    TOFSensor sensor; 
    final MotionMagicVoltage motMag;

    public Intake() {
        driveNeo = new CANSparkMax(Constants.Intake.DRIVE_MOTOR, MotorType.kBrushless);
        driveNeo.setIdleMode(IdleMode.kBrake);
        swivelFalcon = new TalonFX(Constants.Intake.SWIVEL_MOTOR);

        sensor = new TOFSensor(Constants.Intake.INTAKE_SENSOR_ID);

        motMag = new MotionMagicVoltage(0);
        motMag.Slot = 0;
        var talonFXConfigs = new TalonFXConfiguration();

        swivelFalcon.getConfigurator().apply(new TalonFXConfiguration()); // set factory default

        talonFXConfigs.Slot0.kV = Constants.Intake.SWIVEL_KV * 2048 / 1023;

        talonFXConfigs.Slot0.kP = Constants.Intake.SWIVEL_KP * 2048 / 1023; // per new phoenix 6 units
        talonFXConfigs.Slot0.kI = Constants.Intake.SWIVEL_KI * 2048 / 1023 * 1000;
        talonFXConfigs.Slot0.kD = Constants.Intake.SWIVEL_KD * 2048 / 1023 / 1000;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.MOTMAGMAXVEL / 2048 * 10; // rps cruise velocity
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Intake.MOTMAGMAXACCEL / 2048 * 10; // rps/s acceleration 
        talonFXConfigs.MotionMagic.MotionMagicJerk = 800; // rps/s^2 jerk 
        
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
        System.out.println("Stopped");
        setDriveIntakeSpeed(0.0);
        this.isRunning = false;
    }

    public boolean isCommandDone(double position) {
        return ((Math.abs(getPosition() - position) < 1.5) && (this.isRunning && this.sensor.getNoteDetected()));
    }

    public Command getPosAndRunIntakeCommand(double position, double speed) {
        return new FunctionalCommand(
            () -> {System.out.println("working intake"); this.driveNeo.set(speed); this.isRunning = true;}, 
            () -> setPosition(position),
            interrupted -> stopEverything(),
            () -> isCommandDone(position),
            this
        );
    }

    public Command getIntakeDriveCommand(double speed) {
        return new FunctionalCommand(
            () -> this.isRunning = true,
            () -> setDriveIntakeSpeed(speed),
            interrupted -> {setDriveIntakeSpeed(0.0); this.isRunning = false;},
            () -> (this.isRunning && this.sensor.getNoteDetected()),
            this
        );
    }


    public Command getIntakeRoutineCommand() {
        return new SequentialCommandGroup(
            getPosAndRunIntakeCommand(Constants.Intake.DOWN_POSITION, 0.5),
            getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0)
        );
    }

    public Command getIntakeFeedCommand(double speed) {
        return new InstantCommand(
            () -> driveNeo.set(speed)
        );
    }


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Swivel Falcon Encoder", getPosition());
    }
}