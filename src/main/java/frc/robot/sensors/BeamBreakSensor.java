package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.Lights;
import frc.robot.Constants.Lights.LightStates;

public class BeamBreakSensor {
    DigitalInput dio;
    Lights lights;

    public BeamBreakSensor(int id, Lights lights) {
        this.dio = new DigitalInput(id);
        this.lights = lights;
    }   

    public boolean getNoteDetected() {
        if (this.dio.get()) {
            lights.setState(LightStates.ORANGE);
        } else {
            lights.setState(LightStates.PURPLE);
        }

        return this.dio.get();
    }
}