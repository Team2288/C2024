package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
    
    private Spark controller;
    private double state;
    public static double DEFAULT;
    public Lights() {
        //controller = new Spark(9);
        
        //state = 0.0;
    }

    public void setState(double state) {
        /*this.state = state;
        if(state == Constants.Lights.PURPLE || state == Constants.Lights.ORANGE) {
            this.DEFAULT = state;
        }*/
    }

    @Override
    public void periodic() {
        //controller.set(state);
        //this.state = this.DEFAULT;
    }
}