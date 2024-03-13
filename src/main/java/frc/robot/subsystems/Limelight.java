package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
    NetworkTable table;
    NetworkTableEntry tx, ty, ta, tl, botpose;
    double x, y, area, latency, distance;
    double mountAngle; //radians
    double mountHeight, speakerTargetHeight, speakerGoalRange; //meters

    Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.0254, 0.584), new Rotation3d(0, 0, 0));
    PhotonCamera limelight;
    PhotonPoseEstimator poseEstimator;
    PhotonPipelineResult result;
    boolean hasTargets;
    AprilTagFieldLayout aprilTagFieldLayout;

    LimelightHelpers.PoseEstimate pose;

    public Limelight() {
        
        table = NetworkTableInstance.getDefault().getTable("limelight-ironman");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tl = table.getEntry("tl");
        table.getValue("");
        
        // Mount information
        mountAngle = Units.degreesToRadians(15.0);
        mountHeight = Units.inchesToMeters(25.125);
        // Height of the targeted april tag
        speakerTargetHeight = Units.inchesToMeters(55.625);
        // Desired range from the speaker
        speakerGoalRange = Units.inchesToMeters(.0);
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        //poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, limelight, robotToCam);
        pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("ironman");
    }

    @Override
    public void periodic() {
        
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
        
        pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("ironman");
        //result = limelight.getLatestResult();
        //hasTargets = result.hasTargets();
    }


    public LimelightHelpers.PoseEstimate getEstimatedAprilTagPose() {
        return this.pose;
    }

    // Find the horizontal distance(meters) from the speaker given
        // mount height(h1) and angle(a1), and target height(h2); a2(y)=angle from limelight to target(given by crosshair)
    public double distanceFromTarget() {
        //tan(a1+a2) = (h2-h1) / d  -->  d = (h2-h1) / tan(a1+a2)
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