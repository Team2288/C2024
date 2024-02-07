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

public class Intake extends SubsystemBase {
    CANSparkMax driveNeo; // motors
    TalonFX swivelFalcon;
    boolean hasNote, isRunning;
    TOFSensor sensor; 
    final MotionMagicVoltage motMag;

    public Intake() {
        driveNeo = new CANSparkMax(Constants.Intake.DRIVE_MOTOR, MotorType.kBrushless);
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
        if (speed != 0.0) {this.isRunning = true;} else {this.isRunning = false;}

        driveNeo.set(speed); 
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

    public Command getIntakeDownCommand(double position) {
        return new FunctionalCommand(
            () -> setDriveIntakeSpeed(1), // at start: turn on intake
            () -> setPosition(position), // go to position
            interrupted -> {setPosition(0); driveNeo.stopMotor();}, 
            () -> Math.abs(getPosition() - position) < 100 && hasNote == true, // if close enough to position and intake has Note, we are done
            this
        );
    }

    public Command getIntakeFeedCommand(double speed) {
        return new InstantCommand(
            () -> driveNeo.set(speed)
        );
    }

    public Command getIntakeUpCommand(double position) {
        return new FunctionalCommand(
            () -> driveNeo.stopMotor(), 
            () -> setPosition(position),
            interrupted -> {setPosition(0); driveNeo.stopMotor();},
            () -> Math.abs(getPosition() - position) < 100 && hasNote == false,
            this
        );
    }

    public Command getIntakeRoutineCommand() {
        return new SequentialCommandGroup(
            getIntakeDownCommand(Constants.Intake.DOWN_POSITION),
            getIntakeUpCommand(Constants.Intake.UP_POSITION)
        );
    }

    @Override
    public void periodic() {
        if (this.isRunning && this.sensor.getNoteDetected()){
            setDriveIntakeSpeed(0.0);
        }

        SmartDashboard.putNumber("Swivel Falcon Encoder", getPosition());
    }
}