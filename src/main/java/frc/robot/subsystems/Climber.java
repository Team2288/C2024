package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {
    TalonFX motor;
    final MotionMagicVoltage motMag;

    public Climber() {
        // Initialize motor, motor controller, and settings
        motor = new TalonFX(Constants.Climber.MOTOR_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);

        motMag = new MotionMagicVoltage(0);
        motMag.Slot = 0;
    
        motor.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;

        talonFXConfigs.Slot0.kV = Constants.Climber.kV * 2048 / 1023;
        talonFXConfigs.Slot0.kP = Constants.Climber.kP * 2048 / 1023; // per new phoenix 6 units
        talonFXConfigs.Slot0.kI = Constants.Climber.kI * 2048 / 1023 * 1000;
        talonFXConfigs.Slot0.kD = Constants.Climber.kD * 2048 / 1023 / 1000;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.MOTMAGMAXVEL / 2048 * 10; // rps cruise velocity
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Climber.MOTMAGMAXACCEL / 2048 * 10; // rps/s acceleration 
        talonFXConfigs.MotionMagic.MotionMagicJerk = 12000; // rps/s^2 jerk 
        
        motor.getConfigurator().apply(talonFXConfigs);
       // motorEncoder.setPosition(0); // !!! Delete after climber calibration
    }

    // Set the speed of the motor (Percent output)
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void setPosition(double rotations) {
        motor.setControl(motMag.withPosition(rotations));
    }

    // Return the current encoder position 
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", this.getPosition());
    }
}