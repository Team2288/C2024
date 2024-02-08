package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.RelativeEncoder; 
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TOFSensor;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shooter extends SubsystemBase {
    CANSparkMax motorLeft, motorRight;
    boolean is_running;
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
        motorLeftController.setI(Constants.Shooter.SHOOTER_KI); motorRightController.setI(Constants.Shooter.SHOOTER_KI);
        motorLeftController.setD(Constants.Shooter.SHOOTER_KD); motorRightController.setD(Constants.Shooter.SHOOTER_KD);
        motorLeftController.setFF(Constants.Shooter.SHOOTER_KF); motorRightController.setFF(Constants.Shooter.SHOOTER_KF);

        motorLeftController.setOutputRange(-1, 1);
        motorRightController.setOutputRange(-1, 1);

        SmartDashboard.putNumber("P gain", Constants.Shooter.SHOOTER_KP);
        SmartDashboard.putNumber("I gain", Constants.Shooter.SHOOTER_KI);
        SmartDashboard.putNumber("D gain", Constants.Shooter.SHOOTER_KD);
        SmartDashboard.putNumber("F Gain", Constants.Shooter.SHOOTER_KF);
    }

    public Command getShooterCommand(double speed) {
        return new InstantCommand(() -> shoot(speed), this);
    } 

    public Command rampVelocityPIDs(double rpm) {
        return new FunctionalCommand(
            () -> System.out.println("Ramping Shooter"),
            () -> this.setVelocity(rpm),
            interrupted -> {},
            () -> (Math.abs(this.motorEncoderLeft.getVelocity() - rpm) < 100),
            this
        );
    }

    public void setVelocity(double rpm) {
        if (rpm > 0) {is_running = true;} else {is_running = false;}

        motorLeftController.setReference(rpm, ControlType.kVelocity);
        motorRightController.setReference(-rpm, ControlType.kVelocity);
    }

    public void shoot(double speed) {
        motorLeft.set(speed);
        motorRight.set(-speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Motor Velocity", motorEncoderLeft.getVelocity());
        SmartDashboard.putNumber("Right Motor Velocity", motorEncoderRight.getVelocity());

        /* 
        double p = SmartDashboard.getNumber("P gain", 0);
        double i = SmartDashboard.getNumber("I gain", 0);
        double d = SmartDashboard.getNumber("D gain", 0);
        double ff = SmartDashboard.getNumber("F Gain", 0);

        if((p != Constants.Shooter.SHOOTER_KP)) { motorLeftController.setP(p); motorRightController.setP(p); Constants.Shooter.SHOOTER_KP = p; }
        if((i != Constants.Shooter.SHOOTER_KI)) { motorLeftController.setI(i); motorRightController.setI(i); Constants.Shooter.SHOOTER_KI = i; }
        if((d != Constants.Shooter.SHOOTER_KD)) { motorLeftController.setD(d); motorRightController.setD(d); Constants.Shooter.SHOOTER_KD = d; }
        if((ff != Constants.Shooter.SHOOTER_KD)) { motorLeftController.setFF(ff); motorRightController.setFF(ff); Constants.Shooter.SHOOTER_KF = ff; }

        this.setVelocity(0.8);
        //motorLeftController.setReference(0.8, ControlType.kVelocity);
        //motorRightController.setReference(0.8, ControlType.kVelocity);
        */
    }
}