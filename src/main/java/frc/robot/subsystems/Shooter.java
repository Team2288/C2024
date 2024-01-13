package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter extends SubsystemBase {
    CANSparkMax motorLeft, motorRight;
    boolean ready;
    DigitalInput sensor;

    public Shooter() {

    }

    @Override
    public void periodic() {
        
    }
}
