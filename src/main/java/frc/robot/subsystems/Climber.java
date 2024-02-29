package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class Climber {
    CANSparkMax motor;
    RelativeEncoder motorEncoder;
    SparkPIDController motorController;

    public Climber() {
        // Initialize motor, motor controller, and settings
        motor = new CANSparkMax(Constants.Climber.MOTOR_ID, MotorType.kBrushless);
        motorEncoder = motor.getEncoder();
        motorController = motor.getPIDController();

        // Set PID values
        motorController.setP(Constants.Climber.kP); 
        motorController.setI(Constants.Climber.kI); 
        motorController.setD(Constants.Climber.kD); 
        motorController.setFF(Constants.Climber.kF); 

        motorController.setOutputRange(-1, 1);
    }

    // Set the speed of the motor (Percent output)
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    // Return the current encoder position 
    public double getPosition() {
        return motorEncoder.getPosition();
    }
}