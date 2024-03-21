package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class AmpAlignSwerve extends Command{
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup, strafeSup, rotationSup;
    private int priorityTag;

    public AmpAlignSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        if(DriverStation.getAlliance().get() == Alliance.Red) {
            priorityTag = 5;
        } else { 
            priorityTag = 6;
        }
        LimelightHelpers.setPriorityTagID("limelight-ironman", priorityTag);
    }

    private Double limelightStrafeVal() {  
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = 0.005;
        double targetingStrafeVelocity = 
            LimelightHelpers.getTX("limelight-ironman") * kP;
        //invert since tx is positive when the target is to the right of the crosshair
        targetingStrafeVelocity *= -1;
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
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.moveDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.moveDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.moveDeadband);

        if(LimelightHelpers.getFiducialID("limelight-ironman") == priorityTag) {
            strafeVal = limelightStrafeVal();
            rotationVal = limelightRotationVal();
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
    }

}
