package frc.robot;
import com.playingwithfusion.TimeOfFlight;

public class TOFSensor {
    TimeOfFlight sensor;

    public TOFSensor(int canid) {
        sensor = new TimeOfFlight(canid);
    }

    public boolean getNoteDetected() {
        if (this.sensor.isRangeValid()) {
            return (this.sensor.getRange() < 50.0);
        } else return false;
    }
}
