package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TOFSensor;
import frc.robot.subsystems.Shooter;

public class Elevator extends SubsystemBase {
    public CANSparkMax driveMotor;
    public double kP, kI, kD, kF, kMaxOutput, kMinOutput;
    private TalonFX elevatorMotor;
    public ArrayList<String[]> loggingData;
    final MotionMagicVoltage motMag;
    TOFSensor sensor;

    public Elevator() {
        super();

        // Initialize motors, motor controllers, and settings
        driveMotor = new CANSparkMax(Constants.Elevator.DRIVE_MOTOR_ID, MotorType.kBrushless);
        driveMotor.setClosedLoopRampRate(.3);

        elevatorMotor = new TalonFX(Constants.Elevator.POSITION_MOTOR_ID);
    

        motMag = new MotionMagicVoltage(0);
        motMag.Slot = 0;
        var talonFXConfigs = new TalonFXConfiguration();

        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration()); // set factory default

        talonFXConfigs.Slot0.kV = Constants.Elevator.ELEVATOR_KV;

        talonFXConfigs.Slot0.kP = Constants.Elevator.ELEVATOR_KP; 
        talonFXConfigs.Slot0.kI = Constants.Elevator.ELEVATOR_KI;
        talonFXConfigs.Slot0.kD = Constants.Elevator.ELEVATOR_KD;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MOTMAGMAXVEL; // rps cruise velocity
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MOTMAGMAXACCEL; // rps/s acceleration 
        talonFXConfigs.MotionMagic.MotionMagicJerk = 3200; // rps/s^2 jerk 
        
        elevatorMotor.getConfigurator().apply(talonFXConfigs, 0.050);
    }

    public void setElevatorSpeed(double speed) {
        driveMotor.set(speed);
    }

    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    public void setElevatorPosition(double rotations) {
        elevatorMotor.setControl(motMag.withPosition(rotations));
    }
    
    public void SmartDashboard() {
        SmartDashboard.putNumber("Position", elevatorMotor.getPosition().getValue());
    }

    public Command getElevatorAmpRoutineCommand(Shooter s_Shooter, Intake s_Intake) {
        return new SequentialCommandGroup(
            new FunctionalCommand( // on init, set both shooter and intake speed and move to get the note
                () -> s_Intake.setDriveIntakeSpeed(Constants.Intake.SPEED),
                () -> { // on exec
                    this.setElevatorPosition(Constants.Elevator.UP1);
                    s_Shooter.setVelocityVoltageBased(0.09 * 1.5);
                    this.setElevatorSpeed(Constants.Elevator.SPEED * 1.5);
                },
                interrupted -> {
                    this.setElevatorSpeed(0.0); 
                    s_Shooter.setVelocityVoltageBased(0.0);
                    s_Intake.setDriveIntakeSpeed(0.0);
                }, // ended -> shut off shooter, intake, and elevator
                () -> Math.abs(this.getPosition() - Constants.Elevator.UP1) < 4 && this.sensor.getNoteDetected(), // shut down if we're at the shooter AND the sensor has a note
                            // I changed the error to 3 rotations, 12 is way too much -mo
                this,
                s_Shooter,
                s_Intake
            ),
            getElevatorPositionCommand(Constants.Elevator.UP2), // go to the amp
            new FunctionalCommand( // run intake, when note is not there anymore set speed to 0 and return to default position
                () -> this.setElevatorSpeed(Constants.Elevator.SPEED),
                () -> {},
                interrupted -> {this.setElevatorSpeed(0.0); this.setElevatorPosition(Constants.Elevator.DOWN);},
                () -> !this.sensor.getNoteDetected(),
                this
            )
        );
    }

    public Command getElevatorPositionCommand(double rotations) {
        return new FunctionalCommand(
            () -> System.out.println("Running elevator"),
            () -> setElevatorPosition(rotations),
            interrupted -> {},
            () -> Math.abs(getPosition() - rotations) < 4,
            this
        );
    }

    @Override
    public void periodic() {

    }
}
