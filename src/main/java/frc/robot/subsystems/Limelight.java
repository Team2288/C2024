package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    String limelight = "limelight-ironman";
    double mountAngle; //radians
    double mountHeight, speakerTargetHeight, speakerGoalRange; //meters
    Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.0254, 0.584), new Rotation3d(0, 0, 0));
    LimelightHelpers.PoseEstimate pose;

    NetworkTable table;
    NetworkTableEntry tx, ty, ta, tl, botpose;
    double x, y, area, latency, distance;

    boolean hasTargets;
    AprilTagFieldLayout aprilTagFieldLayout;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight-ironman");
        /*
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tl = table.getEntry("tl");
        */

        // Mount information
        mountAngle = Units.degreesToRadians(15.0);
        mountHeight = Units.inchesToMeters(25.125);
        // Height of the targeted april tag
        speakerTargetHeight = Units.inchesToMeters(55.625);
        // Desired range from the speaker
        speakerGoalRange = Units.inchesToMeters(.0);
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
    }

    @Override
    public void periodic() {
        /*
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        latency = tl.getDouble(0.0);
        if(Math.abs(y) < 0.1){
            distance = 1000.0;
        } else {
            distanceFromTarget();
        }
        area = ta.getDouble(0.0);
        SmartDashboard(); 
        result = limelight.getLatestResult();
        hasTargets = result.hasTargets();
        */
        
        pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
    }

    public LimelightHelpers.PoseEstimate getEstimatedAprilTagPose() {
        return this.pose;
    }
    public double getTX() {
        return LimelightHelpers.getTX(limelight);
    }
    public double getTY() {
        return LimelightHelpers.getTY(limelight);
    }

    // Find the horizontal distance(meters) from the speaker given
        // mount height(h1) and angle(a1), and target height(h2); a2(y)=angle from limelight to target(given by crosshair)
    public double distanceFromTarget() {
        // tan(a1+a2) = (h2-h1) / d  -->  d = (h2-h1) / tan(a1+a2)
        double angleToGoalRad = (mountAngle + y) * (Math.PI / 180);
        return distance = (speakerTargetHeight - mountHeight) / Math.tan(angleToGoalRad);
    }

    // Post to smart dashboard
    public void SmartDashboard() {
        /*
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("Pose", pose);
        */
    } 
}