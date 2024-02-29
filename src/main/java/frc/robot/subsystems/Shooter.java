package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.RelativeEncoder; 
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TOFSensor;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase {
    boolean is_running;
    private TalonFX motorRight, motorLeft;
    final VelocityVoltage velControl;
    private CurrentLimitsConfigs currentLimits;
    private Lights s_Lights;

    public Shooter() {
        motorLeft = new TalonFX(Constants.Shooter.LEFT_MOTOR_ID);
        motorRight = new TalonFX(Constants.Shooter.RIGHT_MOTOR_ID);

        velControl = new VelocityVoltage(0);
        velControl.Slot = 0;
        var talonFXConfigs = new TalonFXConfiguration();
        currentLimits = new CurrentLimitsConfigs();

        currentLimits.StatorCurrentLimit = 40;
        currentLimits.StatorCurrentLimitEnable = true;

        motorLeft.getConfigurator().apply(new TalonFXConfiguration()); // set factory default
        motorRight.getConfigurator().apply(new TalonFXConfiguration()); // set factory default

        talonFXConfigs.CurrentLimits = currentLimits;

        motorLeft.getConfigurator().apply(talonFXConfigs, 0.050);
        motorRight.getConfigurator().apply(talonFXConfigs, 0.050);
    }

    public boolean isFinishedRamping(double pctspeed) {
        return Math.abs(this.motorRight.getVelocity().getValueAsDouble() - (pctspeed * 1200 / 60)) < 300;
    }

    public void shootWhenClose(Limelight limelight, double pctspeed) {
        if(limelight.distanceFromTarget() < 200 && limelight.distanceFromTarget() > 10) {
            setSpeed(pctspeed);
            s_Lights.shooting();
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
        this.motorRight.set(pctspeed);
    }

    public void setVelocity(double rpm) {
        if (rpm > 0) {is_running = true;} else {is_running = false;}


        motorLeft.setControl(velControl.withVelocity(rpm));
        motorRight.setControl(velControl.withVelocity(rpm));
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