package frc.robot;
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
        if (this.sensor.isRangeValid()) {
            return (this.sensor.getRange() < 50.0); // distance (millimeters)
        } else return false;
    }

    public void SmartDashboard() {
        SmartDashboard.putNumber("Sensor Range", getRangeDebugger());
    }
}
