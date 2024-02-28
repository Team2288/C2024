package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TOFSensor {
    TimeOfFlight sensor;

    public TOFSensor(int canid) {
        sensor = new TimeOfFlight(canid);
    }

    public double getRangeDebugger() {
        return this.sensor.getRange();
    }

    public boolean getNoteDetected() {
        return (this.sensor.getRange() <= 250); // distance (millimeters)
    }

    public void SmartDashboard() {
        SmartDashboard.putNumber("Sensor Range", getRangeDebugger());
    }
}
