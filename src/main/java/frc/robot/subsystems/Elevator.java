package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.TOFSensor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import frc.robot.sensors.BeamBreakSensor;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Elevator extends SubsystemBase {
    public double kP, kI, kD, kF, kMaxOutput, kMinOutput;
    private TalonFX driveMotor, elevatorMotor;
    public ArrayList<String[]> loggingData;
    private CurrentLimitsConfigs currentConfigs;
    final MotionMagicVoltage motMag;
    BeamBreakSensor sensor;

    public Elevator() {
        super();

        // Initialize motors, motor controllers, and settings
        /*
        driveMotor = new CANSparkMax(Constants.Elevator.DRIVE_MOTOR_ID, MotorType.kBrushless);
        driveMotor.setClosedLoopRampRate(.3);
        driveMotor.setSmartCurrentLimit(15);
        */
        driveMotor = new TalonFX(Constants.Elevator.DRIVE_MOTOR_ID);
        elevatorMotor = new TalonFX(Constants.Elevator.POSITION_MOTOR_ID);
    
        motMag = new MotionMagicVoltage(0);
        motMag.Slot = 0;
        var talonFXConfigs = new TalonFXConfiguration();
        currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = 80;
        currentConfigs.StatorCurrentLimitEnable = true;

        driveMotor.getConfigurator().apply(new TalonFXConfiguration()); // set factory default
        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration()); // set factory default

        talonFXConfigs.Slot0.kV = Constants.Elevator.ELEVATOR_KV;
        talonFXConfigs.Slot0.kP = Constants.Elevator.ELEVATOR_KP; 
        talonFXConfigs.Slot0.kI = Constants.Elevator.ELEVATOR_KI;
        talonFXConfigs.Slot0.kD = Constants.Elevator.ELEVATOR_KD;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MOTMAGMAXVEL; // rps cruise velocity
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MOTMAGMAXACCEL; // rps/s acceleration 
        talonFXConfigs.MotionMagic.MotionMagicJerk = 3200; // rps/s^2 jerk 
        
        talonFXConfigs.CurrentLimits = currentConfigs;
        elevatorMotor.getConfigurator().apply(talonFXConfigs, 0.050);
    }

    public void setElevatorSpeed(double speed) {
        driveMotor.set(-speed);
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
            new FunctionalCommand( // on init, run everything
                () -> s_Intake.setDriveIntakeSpeed(Constants.Intake.SPEED),
                () -> { // on exec, go up to the amp
                    this.setElevatorPosition(Constants.Elevator.UP1);
                },
                interrupted -> { // when its interrupted we know the elevator is at the amp, therefore we run the shooter and elevator to pass the note
                    s_Shooter.setSpeed(0.09 * 1.5);
                    this.setElevatorSpeed(Constants.Elevator.SPEED * 1.5);
                }, 
                () -> Math.abs(this.getPosition() - Constants.Elevator.UP2) < 4,
                this,
                s_Shooter,
                s_Intake
            ),
            new WaitCommand(2),
            new InstantCommand( // Shut everything off, bring elevator back down
                () -> {
                    this.setElevatorSpeed(0.0); 
                    s_Shooter.setSpeed(0.0);
                    s_Intake.setDriveIntakeSpeed(0.0);
                    this.setElevatorPosition(Constants.Elevator.DOWN);
                },
                
                this,
                s_Shooter,
                s_Intake
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

    public void setElevatorPositionVoltage(double volt) {
        this.elevatorMotor.set(volt);
    }

    @Override
    public void periodic() {

    }
}
