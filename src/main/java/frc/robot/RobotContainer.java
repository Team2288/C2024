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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
    private final int rotationAxis = 2;
    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    public final TOFSensor sensor = new TOFSensor(20);
    // LEDs (Triggers are the same thing as buttons)
    //private final Trigger shoot = codriver.button(Constants.Buttons.LED_ORANGE);
    //private final Trigger intake = codriver.button(Constants.Buttons.LED_YELLOW);
   // private final Trigger purpleLED = codriver.button(Constants.Buttons.LED_PURPLE);

   private final Trigger intake_on = codriver.button(Constants.Buttons.INTAKE_ON);


    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    // private final Lights s_Lights = new Lights();
    public final Intake s_Intake = new Intake();
    // private final Elevator s_Elevator = new Elevator();
    public final Shooter s_Shooter = new Shooter();

    /* Auto Chooser */

    private SendableChooser<Command> autoChooser;

    /* The container for the robot. Contains subsystems, OI devices, and commands. */
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

        /* 
        s_Lights.setDefaultCommand(
            new LightsCommand(
                () -> s_Lights.off(),
                null
            )
        );

        */

        s_Shooter.setDefaultCommand( // the default command is not to shoot lmao
            new InstantCommand(
                () -> s_Shooter.shoot(0.0)
            )
        );

        s_Intake.setDefaultCommand(
            new InstantCommand(
                () -> s_Intake.flipIntake(Constants.Intake.UP_POSITION) // be neutral
            )
        );

        /* Set Events for Path planning */

        NamedCommands.registerCommand("Intake Routine", new WaitCommand(0)); // s_Intake.getIntakeRoutineCommand()
        NamedCommands.registerCommand("Shoot", new WaitCommand(0)); // s_Shooter.getShooterCommand()
        
        // Auto Chooser

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

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

        intake_on.toggleOnTrue(
            new StartEndCommand(
                () -> {s_Shooter.shoot(1.0); s_Intake.setDriveIntakeSpeed(1.0);},
                () -> {s_Shooter.shoot(0.0); s_Intake.setDriveIntakeSpeed(0.0);},
                s_Intake,
                s_Shooter
            )
        );

        /* 
        orangeLED.toggleOnTrue(
            new LightsCommand( // Press once to turn on lights, press again to turn off lights
                () -> s_Lights.orange(), // First
                () -> s_Lights.off() // Second
            )
        );
        
        yellowLED.toggleOnTrue(
            new LightsCommand(
                () -> s_Lights.yellow(),
                () -> s_Lights.off()
            )
        );

        purpleLED.toggleOnTrue(
            new LightsCommand(
                () -> s_Lights.purple(),
                () -> s_Lights.off()
            )
        );
        */

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return autoChooser.getSelected();
        return new exampleAuto(s_Swerve);
    }
}
