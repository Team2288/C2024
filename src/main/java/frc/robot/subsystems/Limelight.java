package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    NetworkTable table;
    NetworkTableEntry tx, ty, ta, botpose;
    double x, y, area, pose, distance;
    double mountAngle; //degree
    double mountHeight, targetHeight; //inches

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight-ironman");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        table.getValue("");
        mountAngle = 15.0;
        mountHeight = 25.125;
        targetHeight = 55.625;

        botpose = table.getEntry("botpose");
    }

    //read values periodically
    @Override
    public void periodic() {
        
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        if(x == 0.0 && y == 0.0){
            distance = 0;
        } else {
            distanceFromTarget();
        }
        area = ta.getDouble(0.0);
        pose = botpose.getDouble(0.0);
        SmartDashboard(); 
    }

    // find the horizontal distance(inches) from a target given
        // mount height(h1) and angle(a1), and target height(h2); a2(y)=angle from limelight to target(given by crosshair)
    public double distanceFromTarget() {
        //tan(a1+a2) = (h2-h1) / d  -->  d = (h2-h1) / tan(a1+a2)
        double angleToGoalRad = (mountAngle + y) * (Math.PI / 180);
        return distance = (targetHeight - mountHeight) / Math.tan(angleToGoalRad);
    }

    // post to smart dashboard
    public void SmartDashboard() {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("Pose", pose);
    } 
}