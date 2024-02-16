package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final CommandXboxController codriver = new CommandXboxController(1); 

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = 2;
    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    // LEDs (Triggers are the same thing as buttons)
    //private final Trigger shoot = codriver.button(Constants.Buttons.LED_ORANGE);
    //private final Trigger intake = codriver.button(Constants.Buttons.LED_YELLOW);
    //private final Trigger purpleLED = codriver.button(Constants.Buttons.LED_PURPLE);

    private final Trigger intake_on = codriver.a();
    private final Trigger shoot = codriver.x();

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    // private final Lights s_Lights = new Lights();
    public final Intake s_Intake = new Intake();
    // private final Elevator s_Elevator = new Elevator();
    public final Shooter s_Shooter = new Shooter();
    public final Climber s_Climber = new Climber();

    /* Auto Chooser */

    private SendableChooser<Command> autoChooser;
    Command auto;
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
        s_Intake.setDefaultCommand(
            new FunctionalCommand(
                () -> System.out.println("Default intake command initialized"),
                () -> s_Intake.setPosition(Constants.Intake.UP_POSITION),
                interrupted -> System.out.println("Command scheduled for Intake"),
                () -> false,
                s_Intake
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
        /*
        s_Shooter.setDefaultCommand( // the default command is not to shoot lmao
            new InstantCommand(
                () -> s_Shooter.shoot(0.0),
                s_Shooter
            )
        );
        */
        /* Set Events for Path planning */

        NamedCommands.registerCommand("IntakeRoutine", s_Intake.getIntakeRoutineCommand()); // s_Intake.getIntakeRoutineCommand()
        NamedCommands.registerCommand("Shoot", new WaitCommand(0)); // s_Shooter.getShooterCommand()
        
        // Auto Chooser


        auto = AutoBuilder.buildAuto("Test");
        // SmartDashboard.putData("Auto Chooser", autoChooser);

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

        intake_on.onTrue(
            this.s_Intake.getIntakeRoutineCommand()
        );

        shoot.onTrue(
            this.getShootCommand()
            // new InstantCommand( () -> s_Shooter.setVelocity(5000))
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

    public Command getIntakeAndShootCommand() {
        return new SequentialCommandGroup(
            this.s_Intake.getIntakeRoutineCommand(),
            this.getShootCommand()
        );
    }

    public Command getShootCommand() {
        return new SequentialCommandGroup(
            s_Shooter.rampVelocityPIDs(5000),
            new WaitCommand(1),
            new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(Constants.Intake.SPEED), s_Intake),
            new WaitCommand(1.5),
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(0.0), s_Intake),
                new InstantCommand(() -> s_Shooter.setVelocity(0), s_Shooter)
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
        // return autoChooser.getSelected();
        return this.auto;
    }
}
