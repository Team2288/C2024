package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SpeakerAlignSwerve extends Command{
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup, strafeSup, rotationSup;
    private double prevErr;
    private int priorityTag;

    public SpeakerAlignSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        prevErr = 0.000001;

        //priorityTag = 4;
        /*
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            priorityTag = 4;
        } else { 
            priorityTag = 7;
        }
        LimelightHelpers.setPriorityTagID("limelight-ironman", priorityTag);
        */
    }

    private Double limelightAimKP() {  
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = 0.005;
        double kD = 0.0000;
        double targetingAngularVelocity = 
            LimelightHelpers.getTX("limelight-ironman") * kP +
            derive(LimelightHelpers.getTX("limelight-ironman")) * kD;
        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1;
        return targetingAngularVelocity;
    }
    private Double limelightRangeProportional() {    
        double kP = .5;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight-ironman") * kP;
        //targetingForwardSpeed = -targetingForwardSpeed;
        return targetingForwardSpeed;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = limelightRangeProportional();
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.moveDeadband) * 4.5;
        double rotationVal = limelightAimKP();

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
    }

    private double derive(double currErr) {
        double derErr = prevErr - currErr;
        prevErr = currErr;
        return derErr;
    }
}

