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

public class Intake extends SubsystemBase {
    CANSparkMax driveNeo, swivelNeo; // motors
    SparkPIDController swivelPID;
    RelativeEncoder neoEncoder;
    ColorSensor intakesensor;
    boolean hasNote;

    public Intake() {
        driveNeo = new CANSparkMax(Constants.Intake.DRIVE_MOTOR, MotorType.kBrushless);
        swivelNeo = new CANSparkMax(Constants.Intake.SWIVEL_MOTOR, MotorType.kBrushless);
        swivelPID = swivelNeo.getPIDController();
        neoEncoder = swivelNeo.getEncoder();

        swivelPID.setP(Constants.Intake.SWIVEL_KP);
        swivelPID.setI(Constants.Intake.SWIVEL_KI);
        swivelPID.setD(Constants.Intake.SWIVEL_KD);
        swivelPID.setOutputRange(-1, 1);

        intakesensor = new ColorSensor();
        hasNote = false;
    }

    public void setDriveIntakeSpeed(int speed) {
        driveNeo.set(speed);
    }

    public void flipIntake(double position) {
        swivelPID.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return neoEncoder.getPosition();
    }
    
    public Command getintakeDownCommand(double position) {
        return new FunctionalCommand(
            () -> setDriveIntakeSpeed(1), // at start: turn on intake
            () -> flipIntake(position), // go to position
            interrupted -> {flipIntake(0); driveNeo.stopMotor();}, 
            () -> Math.abs(getPosition() - position) < 100 && hasNote == true, // if close enough to position and intake has Note, we are done
            this
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
        hasNote = intakesensor.getNoteDetected();
    }
}
