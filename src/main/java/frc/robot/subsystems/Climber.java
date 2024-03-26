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

public class Climber extends SubsystemBase {
    CANSparkMax motor;
    RelativeEncoder motorEncoder;
    SparkPIDController motorController;

    public Climber() {
        // Initialize motor, motor controller, and settings
        motor = new CANSparkMax(Constants.Climber.MOTOR_ID, MotorType.kBrushless);
    
        motor.setIdleMode(IdleMode.kBrake);
        motorEncoder = motor.getEncoder();
        motorController = motor.getPIDController();

        motor.setIdleMode(IdleMode.kBrake);

       // motorEncoder.setPosition(0); // !!! Delete after climber calibration

        // Set PID Controller Values
        motorController.setP(Constants.Climber.kP, 0); 
        motorController.setI(Constants.Climber.kI, 0); 
        motorController.setD(Constants.Climber.kD, 0); 
        motorController.setFF(Constants.Climber.kF, 0); 

        motorController.setP(Constants.Climber.lkP, 1); 
        motorController.setI(Constants.Climber.lkI, 1); 
        motorController.setD(Constants.Climber.lkD, 1); 
        motorController.setFF(Constants.Climber.lkF, 1); 

        motorController.setOutputRange(-1, 1);
    }

    // Set the speed of the motor (Percent output)
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void setPosition(double ticks) {
        if (Math.abs(ticks - Constants.Climber.DOWN_POSITION) < .5) {
            motorController.setReference(ticks, ControlType.kPosition, 1);
        } else {
            motorController.setReference(ticks, ControlType.kPosition, 0);
        }
    }

    // Return the current encoder position 
    public double getPosition() {
        return motorEncoder.getPosition();
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", this.getPosition());
    }
}