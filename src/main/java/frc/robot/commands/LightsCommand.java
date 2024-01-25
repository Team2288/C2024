package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Lights;


public class LightsCommand extends StartEndCommand {
    private Lights s_Lights;
    
    public LightsCommand(Runnable onInit, Runnable onEnd) {
        super(onInit, onEnd);
        addRequirements(s_Lights);
    }

    @Override
    public void initialize(){
        s_Lights.off();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
