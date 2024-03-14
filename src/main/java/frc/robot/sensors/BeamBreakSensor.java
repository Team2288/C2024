package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.Lights;
import frc.robot.Constants.Lights.LightStates;


public class BeamBreakSensor {
    DigitalInput dio;
    private frc.robot.subsystems.Lights lights;

    public BeamBreakSensor(int id, frc.robot.subsystems.Lights lights) {
        this.dio = new DigitalInput(id);
        this.lights = lights;
    }   
    public boolean getNoteDetected() {
        return this.dio.get();
    }

    public void lightsNoteDetected(){
        if (getNoteDetected())
        {this.setState(Constants.Lights.LightStates.GREEN);
        }          
        else {this.setState(Constants.Lights.LightStates.PURPLE);
        }
    }

    private void setState(LightStates state) {
    this.lights.setState(state);
    }
}