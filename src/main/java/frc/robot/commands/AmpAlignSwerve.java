package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AmpAlignSwerve extends Command{
    /*
    private Swerve s_Swerve;  
    private Limelight s_Limelight;  
    private DoubleSupplier translationSup, strafeSup, rotationSup;

    public AmpAlignSwerve(Limelight s_Limelight, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.s_Limelight = s_Limelight;
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }

    private Double limelightStrafeVal() {  
        double kP = 0.06;
        double targetingStrafeVelocity = 
            s_Limelight.getTX() * kP;
        return targetingStrafeVelocity;
    }

    private Double limelightRotationVal() {    
        double kP = .0001;
        double targetRot = 90.0; // check if same for red/blue alliance
        double[] pose = LimelightHelpers.getBotPose_wpiBlue("limelight-ironman");
        double currRot = pose[5];
        return (targetRot - currRot) * kP;
        // targetRot - currRot = error
    }
    

    @Override
    public void execute() {
        /* Get Values, Deadband*
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.moveDeadband) * Constants.Swerve.maxSpeed;
        double strafeVal = limelightStrafeVal();
        double rotationVal = limelightRotationVal();

        /* Drive *
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
    }
    */
}
