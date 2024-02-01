package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import java.util.ArrayList;
import java.lang.System;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public CANSparkMax leader, follower;
    private SparkPIDController leaderController;
    private RelativeEncoder leaderEncoder, followerEncoder;
    public double kP, kI, kD, kF, kMaxOutput, kMinOutput;

    public ArrayList<String[]> loggingData;

    public Elevator() {
        super();
        // Initialize motors, motor controllers, and settings
        leader = new CANSparkMax(Constants.Elevator.LEAD_MOTOR_ID, MotorType.kBrushless);
        follower = new CANSparkMax(Constants.Elevator.FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        leaderController = leader.getPIDController();
        leaderEncoder = leader.getEncoder();
        leaderEncoder.setPositionConversionFactor(100);
        leader.setClosedLoopRampRate(.3);

        follower.follow(leader, true);
        followerEncoder = follower.getEncoder();

        leaderController.setP(Constants.Elevator.ELEVATOR_KP, 0);
        leaderController.setI(Constants.Elevator.ELEVATOR_KI, 0);
        leaderController.setD(Constants.Elevator.ELEVATOR_KD, 0);
    }

    @Override
    public void periodic() {

    }

    public void setPosition(double position) {
        leaderController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return leaderEncoder.getPosition();
    }

    // Create base subsystem commands
    public FunctionalCommand getPositionCommand(double position) {
        return new FunctionalCommand(
            () -> System.out.println("Elevator going to pos: " + position),
            () -> setPosition(position),
            interrupted -> System.out.println("Ended driving elevator"),
            () -> Math.abs(getPosition() - position) <= 300
        );
    }

    // Set encoder values to zero
    public void resetEncoderPosition() {
        leaderEncoder.setPosition(0);
        followerEncoder.setPosition(0);
    }

    public double getOutput() {
        return leader.getOutputCurrent();
    }    
}
