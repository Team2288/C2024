package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import frc.robot.Constants;

public class Lights {
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
    public void setState(LightStates state) {
        try {
            String command = Constants.LIGHT_STATES.getOrDefault(state,Constants.DEFAULT_LIGHT_STATE); //Sets to default setting once an error occurs
            this.port.writeString(command);
            this.port.flush();
        }catch(Exception e) {
            System.err.println("error setting light state for state: " +state.toString());
        }
    }

    public void on() {
        this.setState(LightStates.ON);
    }
    public void off() {
        this.setState(LightStates.OFF);
    }    
    public void yellow(){
        this.setState(LightStates.ON);
        this.setState(LightStates.YELLOW);
    }
    public void purple(){
        this.setState(LightStates.ON);
        this.setState(LightStates.PURPLE);
    }
    public void orange(){
        this.setState(LightStates.ON);
        this.setState(LightStates.ORANGE);
        
    }
}
