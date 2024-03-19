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
        double kP = Constants.Swerve.angleKP;
        double targetingAngularVelocity = LimelightHelpers.getTX("ironman") * kP;
        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity = -targetingAngularVelocity;
        return targetingAngularVelocity;
    }
    private Double limelightRangeProportional() {    
        double kP = Constants.Swerve.driveKP;
        double targetingForwardSpeed = LimelightHelpers.getTY("ironman") * kP;
        targetingForwardSpeed = -targetingForwardSpeed;
        return targetingForwardSpeed;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = limelightRangeProportional();
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.moveDeadband);
        double rotationVal = limelightAimKP();

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
    }
}

