package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Lights;

public class LightCommands extends StartEndCommand{
    public Lights s_Lights;

    public LightCommands(Runnable onInit, Runnable onEnd, Lights requirements) {
        super(onInit, onEnd, requirements);
    }
    
    @Override
    public void initialize(){
        s_Lights.off();
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
