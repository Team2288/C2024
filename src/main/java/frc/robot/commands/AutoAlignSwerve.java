package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class AutoAlignSwerve extends Command{
    private Swerve s_Swerve;    
    private DoubleSupplier strafeSup;

    public AutoAlignSwerve(Swerve s_Swerve, DoubleSupplier strafeSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.strafeSup = strafeSup;
    }

    private Double limelightAimKP() {    
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = 0.005;
        double targetingAngularVelocity = 
            LimelightHelpers.getTX("limelight-ironman") * kP;
        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1;
        return targetingAngularVelocity;
    }
    private Double limelightRangeProportional() {    
        double kP = .4;
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
}

