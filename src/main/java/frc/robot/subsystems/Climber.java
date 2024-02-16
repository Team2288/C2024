package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;

public class Climber {
    CANSparkMax motor;
    RelativeEncoder motorEncoder;
    SparkPIDController motorController;

    public Climber() {
        motor = new CANSparkMax(Constants.Climber.MOTOR_ID, MotorType.kBrushless);

        motorEncoder = motor.getEncoder();
        
        motorController = motor.getPIDController();

        motorController.setP(Constants.Climber.kP); 
        motorController.setI(Constants.Climber.kI); 
        motorController.setD(Constants.Climber.kD); 
        motorController.setFF(Constants.Climber.kF); 

        motorController.setOutputRange(-1, 1);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public double getPosition() {
        return motorEncoder.getPosition();
    }

}