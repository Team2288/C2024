package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;


public class LightsCommand extends Command {
    private Lights s_Lights;

    public LightsCommand(Lights s_Lights){
        this.s_Lights = s_Lights;
        addRequirements(s_Lights);
    }

    @Override
    public void initialize(){
        s_Lights.off();
    }

    @Override
    public void execute(){
        Command setOrange = s_Lights.runOnce(() -> s_Lights.orange());
        Command setPurple = s_Lights.runOnce(() -> s_Lights.purple());
        Command setYellow = s_Lights.runOnce(() -> s_Lights.yellow());
    }
}
