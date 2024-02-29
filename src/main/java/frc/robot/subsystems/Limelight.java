package frc.robot.subsystems;



import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    NetworkTable table;
    NetworkTableEntry tx, ty, ta, botpose;
    double x, y, area, pose, distance;
    double mountAngle; //degree
    double mountHeight, targetHeight; //inches

    PhotonCamera limelight;
    PhotonPipelineResult result;
    boolean hasTargets;

    public Limelight() {
        /*
        table = NetworkTableInstance.getDefault().getTable("limelight-ironman");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        table.getValue("");
        mountAngle = 15.0;
        mountHeight = 25.125;
        targetHeight = 55.625;
        */
        //limelight = new PhotonCamera("photonvision");
        //limelight.setLED(VisionLEDMode.kOn);
    }

    @Override
    public void periodic() {
        /*
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        if(Math.abs(y) < 0.1){
            distance = 1000.0;
        } else {
            distanceFromTarget();
        }
        area = ta.getDouble(0.0);
        pose = botpose.getDouble(0.0);
        SmartDashboard(); 
        */
        //result = limelight.getLatestResult();
        //hasTargets = result.hasTargets();
    }

    // Find the horizontal distance(inches) from a target given
        // mount height(h1) and angle(a1), and target height(h2); a2(y)=angle from limelight to target(given by crosshair)
    public double distanceFromTarget() {
        //tan(a1+a2) = (h2-h1) / d  -->  d = (h2-h1) / tan(a1+a2)
        double angleToGoalRad = (mountAngle + y) * (Math.PI / 180);
        return distance = (targetHeight - mountHeight) / Math.tan(angleToGoalRad);
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