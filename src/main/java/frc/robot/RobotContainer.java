package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.TeleopSwerve;
import frc.robot.Constants.Lights.LightStates;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Lights;

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
//  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
//  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    // LEDs (Triggers are the same thing as buttons)
    //private final Trigger shoot = codriver.button(Constants.Buttons.LED_ORANGE);
    //private final Trigger intake = codriver.button(Constants.Buttons.LED_YELLOW);
    //private final Trigger purpleLED = codriver.button(Constants.Buttons.LED_PURPLE);

    private final Trigger intake_on = codriver.a(); //down
    private final Trigger shoot = codriver.x(); //up
    private final Trigger intakeUp = codriver.y();
    private final Trigger spitNoteOut = codriver.b();
    private final Trigger climber = codriver.rightBumper();
    private final Trigger elevatorClimb = codriver.leftBumper();
    private final Trigger shootAmp = codriver.start();
    private final Trigger backUpShooter = codriver.back();
    private final Trigger slowModeTrigger = new Trigger(() -> driver.getTrigger());

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final Lights s_Lights = new Lights();
    public final Intake s_Intake = new Intake();
    public final Elevator s_Elevator = new Elevator();
    public final Shooter s_Shooter = new Shooter();
    public final Climber s_Climber = new Climber();
    public final Limelight s_Limelight = new Limelight();

    /* Auto Chooser */

    private SendableChooser<Command> autoChooser;
    Command auto;
    private boolean slowMode = false;

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
        */
        // s_Lights.setDefaultCommand(
        //     new LightsCommand(
        //         () -> s_Lights.off(),
        //         null
        //     )
        // );

        
        /*
        s_Shooter.setDefaultCommand( // the default command is not to shoot lmao
            new InstantCommand(
                () -> s_Shooter.shoot(0.0),
                s_Shooter
            )
        );
        */
        /* Set Events for Path planning */

        NamedCommands.registerCommand("IntakeRoutine", this.s_Intake.getIntakeRoutineCommand()); // s_Intake.getIntakeRoutineCommand()
        NamedCommands.registerCommand("Shoot", this.s_Intake.getShootCommandNoRamp()); // s_Shooter.getShooterCommand()
        NamedCommands.registerCommand("IntakeUp", this.s_Intake.getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0));
        NamedCommands.registerCommand("RampUp", new SequentialCommandGroup(this.s_Shooter.rampVelocityPIDs(0.5), new WaitCommand(1)));
        NamedCommands.registerCommand("TurnOffShooter", new InstantCommand(() -> this.s_Shooter.setSpeed(0.0), this.s_Shooter));
        // Auto Chooser
        auto = AutoBuilder.buildAuto("5NoteRight");

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

//        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));            


        intake_on.onTrue(
            //new InstantCommand(() -> s_Elevator.setElevatorPosition(4), s_Elevator)
            this.s_Intake.getIntakeRoutineCommand()
        );

        shoot.onTrue(
            //new InstantCommand(() -> s_Elevator.setElevatorPosition(38), s_Elevator)
            this.getShootCommand()
            // new InstantCommand( () -> s_Shooter.setVelocity(5000))
        );
        

        shootAmp.whileTrue(
            //this.shootAmp()
            new InstantCommand(() -> this.s_Elevator.setElevatorSpeed(0.80), this.s_Elevator)
        );

        
        intakeUp.onTrue(
            this.s_Intake.getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0)
        );

        spitNoteOut.whileTrue(
            this.s_Intake.getIntakeDriveCommand(-0.8)
            //new InstantCommand( () -> this.s_Shooter.setSpeed(0.5), s_Shooter)
        );

        climber.whileTrue(
            new InstantCommand(() -> this.s_Climber.setSpeed(-0.2), s_Climber)
        );

        climber.whileFalse(
            new InstantCommand(() -> this.s_Climber.setSpeed(0.0), s_Climber)
        );

        
        elevatorClimb.onTrue(
            new InstantCommand(() -> this.s_Elevator.setElevatorPosition(60), s_Elevator)
        );


        /* 

        climbDown.whileTrue(
            new InstantCommand(() -> this.s_Climber.setSpeed(-0.2), s_Climber)
        );

        /* 

        elevatorUp.onTrue(
            new SequentialCommandGroup(
                this.s_Elevator.getElevatorPositionCommand(Constants.Elevator.UP1),
                
                new ParallelCommandGroup(
                    new InstantCommand(() -> this.s_Intake.setDriveIntakeSpeed(Constants.Intake.SPEED)),
                    new InstantCommand(() -> this.s_Shooter.setVelocityVoltageBased(0.09 * 1.5), this.s_Shooter),
                    new InstantCommand(() -> this.s_Elevator.setElevatorSpeed(Constants.Elevator.SPEED * 1.5), this.s_Elevator)
                )
            )
        ); 

        elevatorDown.onTrue(
            this.s_Elevator.getElevatorPositionCommand(Constants.Elevator.DOWN)
        );

        */


        /* 

        elevatorDown.onTrue(
            this.s_Elevator.getElevatorPositionCommand(0)
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
            this.s_Intake.getIntakeRoutineCommand()
        );
    }
    public Command shootAmp() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setSpeed(0.09 * 1.5), s_Shooter),
                s_Elevator.getElevatorPositionCommand(Constants.Elevator.UP_AMP)
            ),
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Elevator.setElevatorSpeed(Constants.Elevator.SPEED), s_Elevator),
                new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(Constants.Intake.SPEED), s_Intake)
            ),
            new WaitCommand(2),
            //this.s_Lights.getLightsCommand(LightStates.PURPLE),
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Elevator.setElevatorSpeed(0.0), s_Elevator),
                new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(0.0), s_Intake),
                new InstantCommand(() -> s_Shooter.setSpeed(0), s_Shooter)
            ),
            s_Elevator.getElevatorPositionCommand(Constants.Elevator.DOWN)
        );
    }

    public Command rampCommand() {
        return new SequentialCommandGroup(
            this.s_Shooter.rampVelocityPIDs(0.5),
            new WaitCommand(.7)
        );
    }

    public Command getShootCommand() {
        return new SequentialCommandGroup(
            // this.s_Elevator.getElevatorPositionCommand(-120),
             //new InstantCommand(() -> this.s_Elevator.setElevatorSpeed(Constants.Elevator.SPEED * 1.5), this.s_Elevator),
             //this.s_Shooter.rampVelocityPIDsDifferentSpeeds(0.15, 0.15), // bottom top,
             this.s_Shooter.rampVelocityPIDs(0.5),
             new WaitCommand(1.0),
             new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(Constants.Intake.SPEED), s_Intake),
             new WaitCommand(.7),
             new ParallelCommandGroup(
                 new InstantCommand(() -> this.s_Elevator.setElevatorSpeed(0), this.s_Elevator),
                 new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(0.0), s_Intake),
                 new InstantCommand(() -> s_Shooter.setSpeed(0), s_Shooter)
                 //this.s_Lights.getLightsCommand(LightStates.PURPLE)
             )
         );
 
    }


/*     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return autoChooser.getSelected();
        return this.auto;
    }
}
