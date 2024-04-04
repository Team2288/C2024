package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class SpeakerAlignSwerve extends Command{
    private Limelight s_Limelight;
    private Swerve s_Swerve;    
    private Lights lights;
    private DoubleSupplier translationSup, strafeSup, rotationSup;

    public SpeakerAlignSwerve(Limelight s_Limelight, Swerve s_Swerve, Lights lights, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.s_Limelight = s_Limelight;
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.lights = lights;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }

    private Double limelightAimKP() {  
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = 0.005;
        double targetingAngularVelocity = s_Limelight.getTX() * kP;
        //invert since tx is positive when the target is to the right of the crosshair
        return -targetingAngularVelocity;
    }
    private Double limelightRangeProportional() {    
        double kP = .5;
        double targetingForwardSpeed = s_Limelight.getTY() * kP;
        return targetingForwardSpeed;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = limelightRangeProportional();
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.moveDeadband) * Constants.Swerve.maxSpeed;
        double rotationVal = limelightAimKP();
        if(Math.abs(s_Limelight.getTY()) < 3.5 && Math.abs(s_Limelight.getTX()) < 2) {
            lights.setState(Constants.Lights.GREEN);
        } else {
            lights.setState(Lights.DEFAULT);
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
    }
}

