package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants;

public class Lights extends SubsystemBase {
    private SerialPort port;
      
    public Lights() {
        super();

        this.port = new SerialPort(115200, SerialPort.Port.kOnboard,8,Parity.kEven,StopBits.kOne);
    }
    
    // Set state until state is set otherwise
    public void setState(Constants.Lights.LightStates state) {
        String command = Constants.Lights.HASHMAP_LIGHT_STATES.getOrDefault(state, Constants.Lights.DEFAULT_LIGHT_STATE);
        this.port.writeString(command);
        this.port.flush();
    }

    public FunctionalCommand getLightsCommand(Constants.Lights.LightStates state) {
        return new InstantCommand(
            () -> {this.on(); this.setState(state);},
            this
        );
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
    public void periodic() {
        /* 
        if(limelight.distanceFromTarget() < 250 && limelight.distanceFromTarget() >= 200) {
            this.setState(Constants.Lights.LightStates.SEG1);
        } else if(limelight.distanceFromTarget() < 200 && limelight.distanceFromTarget() >= 150) {
            this.setState(Constants.Lights.LightStates.SEG2);
        } else if(limelight.distanceFromTarget() < 150 && limelight.distanceFromTarget() >= 100) {
            this.setState(Constants.Lights.LightStates.SEG3);
        } else if(limelight.distanceFromTarget() < 100 && limelight.distanceFromTarget() >= 50) {
            this.setState(Constants.Lights.LightStates.SEG4);
        } else {
            this.setState(Constants.Lights.LightStates.OFF);
        }
        */
    }
}