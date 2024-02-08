package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.RelativeEncoder; 
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

public class Shooter extends SubsystemBase {
    CANSparkMax motorLeft, motorRight;
    boolean ready;
    RelativeEncoder motorEncoderLeft, motorEncoderRight;
    SparkPIDController motorLeftController, motorRightController;

    public Shooter() {
        motorLeft = new CANSparkMax(Constants.Shooter.LEFT_MOTOR_ID, MotorType.kBrushless);
        motorRight = new CANSparkMax(Constants.Shooter.RIGHT_MOTOR_ID, MotorType.kBrushless);

        motorEncoderLeft = motorLeft.getEncoder();
        motorEncoderRight = motorRight.getEncoder();

        motorLeftController = motorLeft.getPIDController();
        motorRightController = motorRight.getPIDController(); 

        motorLeftController.setP(Constants.Shooter.SHOOTER_KP); motorRightController.setP(Constants.Shooter.SHOOTER_KP);
        motorLeftController.setFF(Constants.Shooter.SHOOTER_KF); motorRightController.setFF(Constants.Shooter.SHOOTER_KF);
    }

    public Command getShooterCommand(double speed) {
        return new InstantCommand(() -> shoot(speed));
    } 

    public void setVelocity(double vel) {
        motorLeftController.setReference(vel, ControlType.kVelocity);
        motorRightController.setReference(vel, ControlType.kVelocity);
    }

    public void shoot(double speed) {
        motorLeft.set(speed);
        motorRight.set(-speed);
    }

    @Override
    public void periodic() {
        double motorLeft_velocity = motorEncoderLeft.getVelocity();
        double motorRight_velocity = motorEncoderLeft.getVelocity();
        double motorLeft_position = motorEncoderLeft.getPosition();
        double motorRight_position =  motorEncoderRight.getPosition();
        SmartDashboard.putNumber("motor Left voltage", motorLeft.getBusVoltage());
        SmartDashboard.putNumber("motor Right voltage", motorRight.getBusVoltage());
        SmartDashboard.putNumber("motor left Velocity", motorLeft_velocity);
        SmartDashboard.putNumber("motor right Velocity", motorRight_velocity);
        SmartDashboard.putNumber("Get right motor position", motorLeft_position);
        SmartDashboard.putNumber("Get left motor position", motorRight_position);
        //get motor velocities and positions with encoders
        //get voltages
    }
}//frogger!!!