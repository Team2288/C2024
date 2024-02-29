package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private TalonFX motorRight, motorLeft;
    final VelocityVoltage velControl;
    private CurrentLimitsConfigs currentLimits;
    boolean is_running;

    public Shooter() {
        // Initialize motors, motor controllers, and motor settings
        motorLeft = new TalonFX(Constants.Shooter.LEFT_MOTOR_ID);
        motorRight = new TalonFX(Constants.Shooter.RIGHT_MOTOR_ID);
        velControl = new VelocityVoltage(0);
        velControl.Slot = 0;

        // Set factory defaults
        motorLeft.getConfigurator().apply(new TalonFXConfiguration());
        motorRight.getConfigurator().apply(new TalonFXConfiguration());

        // Create and implement current limiter in configs
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 60; //amps
        currentLimits.StatorCurrentLimitEnable = true;
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.CurrentLimits = currentLimits;

        // Set motion control settings
        motorLeft.getConfigurator().apply(talonFXConfigs, 0.050);
        motorRight.getConfigurator().apply(talonFXConfigs, 0.050);

        motorRight.setControl(new Follower(16, false));
    }

    public boolean isFinishedRamping(double pctspeed) {
        return Math.abs(this.motorRight.getVelocity().getValueAsDouble() - (pctspeed * 1200 / 60)) < 300;
    }

    public void shootWhenClose(Limelight limelight, double pctspeed) {
        if(limelight.distanceFromTarget() < 250 && !(Math.abs(limelight.distanceFromTarget() - 113.8275) < 0.01)) {
            //Second condition added because 113.8275 is the default distance due to the mount angle of the limelight
            //that distance causes the shooter to run when the april tag flickers in and out of view
            setSpeed(pctspeed);
            System.out.println("Distance: " + limelight.distanceFromTarget());
        } else {
            setSpeed(0.0);
        }
    }

    public Command rampVelocityPIDs(double pctspeed) {
        return new FunctionalCommand(
            () -> System.out.println("Ramping Shooter"),
            () -> this.setSpeed(pctspeed),
            interrupted -> {},
            () -> isFinishedRamping(pctspeed), // 
            this
        );
    }

    public Command rampVelocityPIDsDifferentSpeeds(double pctspeedright, double pctspeedleft) {
        return new FunctionalCommand(
            () -> System.out.println("Ramping Shooter"),
            () -> {this.motorLeft.set(pctspeedleft); this.motorRight.set(pctspeedright);},
            interrupted -> {},
            () -> isFinishedRamping(pctspeedright), // 
            this
        );
    }

    public void setSpeed(double pctspeed) {
        this.motorLeft.set(pctspeed);
    }

    public void setVelocity(double rpm) {
        if (rpm > 0) {is_running = true;} else {is_running = false;}


        motorLeft.setControl(velControl.withVelocity(rpm));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Motor Velocity", motorLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Velocity", motorRight.getVelocity().getValueAsDouble());

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