package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
    private SerialPort port;

    public Lights() {
        super();

        try {
            this.port = new SerialPort(115200, SerialPort.Port.kUSB1,8,Parity.kEven,StopBits.kOne);
        }catch(Exception e){
            System.err.println("Error setting up serial port, lights will probably not work");
        }
    }
    /*
     * set state until state is set otherwise
     */
    public void setState(Constants.Lights.LightStates state) {
        try {
            String command = Constants.Lights.HASHMAP_LIGHT_STATES.getOrDefault(state, Constants.Lights.DEFAULT_LIGHT_STATE);
            this.port.writeString(command);
            this.port.flush();
        }catch(Exception e) {
            System.err.println("error setting light state for state: " + state.toString());
        }
    }

    public void on() { this.setState(Constants.Lights.LightStates.ON); } 
    
    public void off() { this.setState(Constants.Lights.LightStates.OFF); }    

    public void yellow(){
        on();
        this.setState(Constants.Lights.LightStates.YELLOW);
    }

    public void purple(){
        on();
        this.setState(Constants.Lights.LightStates.PURPLE);
    }

    public void orange(){
        on();
        this.setState(Constants.Lights.LightStates.ORANGE);
    }

    @Override
    public void periodic(){
        //get current light state
    }
}
