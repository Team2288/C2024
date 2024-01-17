package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.io.IOException;
import java.lang.System;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public CANSparkMax leader, follower;
    private SparkMaxPIDController leaderController;
    private RelativeEncoder leaderEncoder, followerEncoder;
    public double kP, kI, kD, kF, kMaxOutput, kMinOutput;

    public double goingTo = 0;
    public Boolean isDone = false;

    public Logger logger;
    public ArrayList<String[]> loggingData;

    public Elevator() {
        super();
        // Initialize motors, motor controllers, and settings
        leader = new CANSparkMax(Constants.LEAD_ELEVATOR_ID, MotorType.kBrushless);
        follower = new CANSparkMax(Constants.FOLLOW_ELEVATOR_ID, MotorType.kBrushless);

        leaderController = leader.getPIDController();
        leaderEncoder = leader.getEncoder();
        leaderEncoder.setPositionConversionFactor(100);
        leader.setClosedLoopRampRate(.3);

        follower.follow(leader, true);
        followerEncoder = follower.getEncoder();

        leaderController.setP(Constants.e_KP, 0);
        leaderController.setI(Constants.e_KI, 0);
        leaderController.setD(Constants.e_KD, 0);
        leaderController.setFF(Constants.e_KF, 0);
        leaderController.setOutputRange(Constants.e_OUTPUT_MIN, Constants.e_OUTPUT_MAX, 0);

        // Initialize logger
        logger = new Logger("Elevator");
        loggingData = new ArrayList<>();
        try {
            logger.initialize();
        } catch (IOException e) {
            System.out.println("IO Exception in Elevator subsystem constructor");
        }

    }

    // Update periodic values
    @Override
    public void periodic() {
        loggingData.add(new String[]{
            logger.getSystemTime(), 
            String.valueOf(leaderEncoder.getPosition()),
            String.valueOf(this.goingTo),
            String.valueOf(this.isDone),
            String.valueOf(leader.getOutputCurrent())
        });

        try {
            logger.log(loggingData);
            loggingData.clear();
        } catch (IOException exception) {
            System.out.println("IO Exception in Subsystem Elevator: " + exception);
        }
    }

    // Position control
    public void setPosition(double position) {
        leaderController.setReference(position, CANSparkMax.ControlType.kPosition);
    }
    public double getPosition() {
        return leaderEncoder.getPosition();
    }

    // Create base subsystem commands
    public FunctionalCommand getPositionCommand(double position) {
        return new FunctionalCommand(
            () -> goingTo = position,
            () -> setPosition(position),
            interrupted -> System.out.println("Ended driving elevator"),
            () -> Math.abs(getPosition() - position) <= 300
        );
    }
    public SequentialCommandGroup getPositionCommandSequential(double... positions) {
        SequentialCommandGroup group = new SequentialCommandGroup(new InstantCommand(() -> setPosition(positions[0])));
    
        for (int i = 1; i < positions.length; i++) {
            final int j = i;
            group.addCommands(
                new InstantCommand(() -> setPosition(positions[j]))
            );
        }

        return group;
    }

    // Set encoder values to zero
    public void resetEncoderPosition() {
        leaderEncoder.setPosition(0);
        followerEncoder.setPosition(0);
    }

    // Put values into SmartDashboard for testing
    public void smartDashboard() {
        SmartDashboard.putNumber("Lead Position", leaderEncoder.getPosition());
    }

    // Experimental Methods
    public void setPositionRoutine(double... positions) {
        for (double position : positions) {
            setPosition(position);
        }
    }

    public double getOutput() {
        return leader.getOutputCurrent();
    }    
}