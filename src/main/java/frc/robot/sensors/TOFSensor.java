package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Lights;

public class TOFSensor {
    TimeOfFlight sensor;
    Lights s_Lights;

    public TOFSensor(int canid) {
        sensor = new TimeOfFlight(canid);
    }

    public double getRangeDebugger() {
        return this.sensor.getRange();
    }

    public boolean getNoteDetected() {
        s_Lights.holdingNote();
        return (this.sensor.getRange() <= 250); // distance (millimeters)
    }

    public void SmartDashboard() {
        SmartDashboard.putNumber("Sensor Range", getRangeDebugger());
    }
}
