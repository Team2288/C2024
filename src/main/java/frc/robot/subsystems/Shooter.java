package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.ColorSensor;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Shooter extends SubsystemBase {
    CANSparkMax motorLeft, motorRight;
    boolean ready;
    ColorSensor shootersensor;

    public Shooter() {
        shootersensor = new ColorSensor();
        motorLeft = new CANSparkMax(Constants.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        motorRight = new CANSParkMax(Constants.Shooter.RIGHT_MOTOR, MotorType.kBrushless);

        motorLeft.follow(motorRight);
    }

    public Command getShooterCommand(double speed) {
        return new FunctionalCommand(
            this.ready = true, // assuming note already loaded
            () -> motorRight.set(speed),
            interrupted -> motorRight.set(0),
            () -> !ready, // finishes when sensor no longer has a note
            this
        );
    } 

    @Override
    public void periodic() {
        ready = shootersensor.getNoteDetected();
    }
}
