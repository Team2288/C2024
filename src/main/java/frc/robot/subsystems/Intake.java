package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.ColorSensor;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Intake extends SubsystemBase {
    CANSparkMax driveNeo, feederNeo; // motors
    RelativeEncoder feederEncoder;
    RelativeEncoder driveEncoder;
    
    TalonFX swivelFalcon;
    boolean hasNote;
    final MotionMagicVoltage motMag;

    public Intake() {
        driveNeo = new CANSparkMax(Constants.Intake.DRIVE_MOTOR, MotorType.kBrushless);
        swivelFalcon = new TalonFX(Constants.Intake.SWIVEL_MOTOR);
        feederNeo = new CANSparkMax(Constants.Intake.FEEDER_MOTOR, MotorType.kBrushless);

        feederEncoder = feederNeo.getEncoder();
        driveEncoder = driveNeo.getEncoder();

        motMag = new MotionMagicVoltage(0);
        motMag.Slot = 0;                            // remember these
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
    }

    public void setDriveIntakeSpeed(double speed) {
        driveNeo.set(speed); feederNeo.set(-speed);
    }

    public void testSwivel(double speed) {
        swivelFalcon.set(speed);
    }

    public void flipIntake(double position) {
        swivelFalcon.setControl(motMag.withPosition(position));
    }

    public double getPosition() { // returns rotations
        swivelFalcon.getRotorPosition().refresh();
        return swivelFalcon.getRotorPosition().getValueAsDouble();
    }

    // Runnable _Consumer _Supplier -> Needs a lambda expression
    public Command getintakeDownCommand(double position) {
        return new FunctionalCommand(
            () -> setDriveIntakeSpeed(1), // at start: turn on intake
            () -> flipIntake(position), // go to position
            interrupted -> {flipIntake(0); driveNeo.stopMotor();}, 
            () -> Math.abs(getPosition() - position) < 100 && hasNote == true, // if close enough to position and intake has Note, we are done
            this
        );
    }

    public Command getIntakeFeedCommand(double speed) {
        return new InstantCommand(
            () -> driveNeo.set(speed)
        );
    }


    public Command getintakeUpCommand(double position) {
        return new FunctionalCommand(
            () -> driveNeo.stopMotor(), 
            () -> flipIntake(position),
            interrupted -> {flipIntake(0); driveNeo.stopMotor();},
            () -> Math.abs(getPosition() - position) < 100 && hasNote == false,
            this
        );
    }

    public Command getIntakeRoutineCommand() {
        return new StartEndCommand(
            () -> getintakeDownCommand(Constants.Intake.DOWN_POSITION),
            () -> getintakeUpCommand(Constants.Intake.UP_POSITION),
            this
        );
    }

    @Override
    public void periodic() {
        double velocity_falcon = swivelFalcon.getVelocity().getValueAsDouble();
        double velocity_feeder = feederEncoder.getVelocity();
        double velocity_drive = driveEncoder.getVelocity();
        double position_drive = driveEncoder.getPosition();
        double position_feeder = feederEncoder.getPosition();
        double voltage = swivelFalcon.getMotorVoltage().getValueAsDouble();
        SmartDashboard.putNumber("Swivel Falcon Encoder", getPosition());
        SmartDashboard.putNumber("DriveNeo position", position_drive);
        SmartDashboard.putNumber("Feeder Position", position_feeder);
        SmartDashboard.putNumber("Swivel falcon Voltage", voltage);
        SmartDashboard.putNumber("DriveNeo Voltage", driveNeo.getBusVoltage());
        SmartDashboard.putNumber("FeederNeo Voltage", feederNeo.getBusVoltage());
        SmartDashboard.putNumber("DriveNeo Volocity", velocity_drive);
        SmartDashboard.putNumber("Swivel Falcon Velocity", velocity_falcon);
        SmartDashboard.putNumber("Feeder Neo Velocity", velocity_feeder);
    }
}
