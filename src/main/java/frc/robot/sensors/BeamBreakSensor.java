package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakSensor {
    DigitalInput dio;

    public BeamBreakSensor(int id) {
        this.dio = new DigitalInput(id);
    }   

    public boolean getNoteDetected() {
        return this.dio.get();
    }
}