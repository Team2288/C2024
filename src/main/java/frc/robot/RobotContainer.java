package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final CommandGenericHID codriver = new CommandGenericHID(1); 

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // LEDs (Triggers are the same thing as buttons)
    private final Trigger orangeLED = codriver.button(Constants.Buttons.LED_ORANGE);
    private final Trigger yellowLED = codriver.button(Constants.Buttons.LED_YELLOW);
    private final Trigger purpleLED = codriver.button(Constants.Buttons.LED_PURPLE);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Lights s_Lights = new Lights();

    // Auto Chooser

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Subsystems

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false // robotCentric.getAsBoolean()
            )
        );

        s_Lights.setDefaultCommand(
            new InstantCommand(
                () -> s_Lights.off(),
                s_Lights
            )
        );

        // Auto Chooser

        autoChooser.setDefaultOption("Example", new exampleAuto(s_Swerve));
        autoChooser.addOption("No Auto", new WaitCommand(0));
        SmartDashboard.putData(autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        orangeLED.toggleOnTrue(
            new StartEndCommand( // StartEndCommand: Press once to turn on lights, press again to turn off lights
                () -> s_Lights.orange(), // First
                () -> s_Lights.off(), // Second
                s_Lights // Subsystem that is needed
            )
        );
        
        yellowLED.toggleOnTrue(
            new StartEndCommand(
                () -> s_Lights.yellow(),
                () -> s_Lights.off(),
                s_Lights
            )
        );

        purpleLED.toggleOnTrue(
            new StartEndCommand(
                () -> s_Lights.purple(),
                () -> s_Lights.off(),
                s_Lights
            )
        );

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}