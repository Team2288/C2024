package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.Lights;
import frc.robot.Constants.Lights.LightStates;

public class BeamBreakSensor {
    DigitalInput dio;

    public BeamBreakSensor(int id) {
        this.dio = new DigitalInput(id);
    }   

    public boolean getNoteDetected() {
        return this.dio.get();
    }
}