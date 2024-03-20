package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SpeakerAlignSwerve;
import frc.robot.commands.TeleopSwerve;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Controllers */
    public final Joystick driver = new Joystick(0);
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
    private final Trigger revShooter = codriver.x(); //up
    private final Trigger shoot = new Trigger(() -> driver.getRawButton(18));
    private final Trigger intakeUp = codriver.y();
    private final Trigger spitNoteOut = codriver.b();
    private final Trigger climberUp = codriver.rightTrigger();
    private final Trigger climberDown = codriver.leftTrigger();
    private final Trigger shootAmp = codriver.start();
    private final Trigger zeroClimber = codriver.rightBumper();
    private final Trigger aim = new Trigger(() -> driver.getRawButton(1));
    private boolean toggle = true;

    /* Subsystems */
    public final Lights s_Lights = new Lights();    
    public final Intake s_Intake = new Intake(s_Lights);
    public final Shooter s_Shooter = new Shooter();
    public final Climber s_Climber = new Climber();
    public final Elevator s_Elevator = new Elevator();
    public final Limelight s_Limelight = new Limelight();
    public final Swerve s_Swerve = new Swerve(s_Limelight);

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
            s_Elevator.setDefaultCommand(
                new FunctionalCommand(
                    () -> System.out.println("Elevator default command initialized"),
                    () -> s_Elevator.setElevatorPosition(Constants.Elevator.DOWN),
                    interrupted -> System.out.println("Command schedule for elevator"),
                    () -> /*(s_Elevator.getPosition() < 3 && s_Elevator.getPosition() > 1)false,
                    s_Elevator
                )
            );*/

            s_Lights.setState(Constants.Lights.LightStates.PURPLE);
        
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

        s_Lights.setDefaultCommand(
            new InstantCommand(
                () -> s_Lights.setState(Constants.Lights.LightStates.PURPLE),
                s_Lights
            )
        );

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

        revShooter
            .onTrue(

                //new InstantCommand(() -> s_Elevator.setElevatorPosition(38), s_Elevator)
                
                this.rampCommand()

                // new InstantCommand( () -> s_Shooter.setVelocity(5000))
            );

        shoot.onTrue(this.getShootCommand());
        
        shootAmp.onTrue(
            this.shootAmp()
        );

        
        intakeUp.onTrue(
            this.s_Intake.getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0)
        );

        spitNoteOut.whileTrue(
            this.s_Intake.getIntakeDriveCommand(-0.8)
            //new InstantCommand( () -> this.s_Shooter.setSpeed(0.5), s_Shooter)
        );

        climberUp.whileTrue(
            new InstantCommand(() -> this.s_Climber.setPosition(Constants.Climber.UP_POSITION), s_Climber)
        );

        climberDown.whileTrue(
            new InstantCommand(() -> this.s_Climber.setPosition(Constants.Climber.DOWN_POSITION), s_Climber)
        );

        zeroClimber.whileTrue(
            new InstantCommand(() -> this.s_Climber.setPosition(0), s_Climber)
        );

        aim.whileTrue(
            new SpeakerAlignSwerve(s_Swerve, () -> -driver.getRawAxis(strafeAxis))
        );

        /* 
        climberUp.whileFalse(
            new InstantCommand(() -> this.s_Climber.setSpeed(0.0), s_Climber)
        );

        
        /*elevatorClimb.onTrue(
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
                new InstantCommand(() -> s_Shooter.setSpeed(0.05), s_Shooter),
                s_Elevator.getElevatorPositionCommand(Constants.Elevator.UP_AMP)
            ),
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Elevator.setElevatorSpeed(Constants.Elevator.SPEED), s_Elevator),
                new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(Constants.Intake.SPEED), s_Intake)
            ),
            new WaitCommand(1),
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Lights.setState(Constants.Lights.LightStates.PURPLE), s_Lights),
                new InstantCommand(() -> s_Elevator.setElevatorSpeed(0.0), s_Elevator),
                new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(0.0), s_Intake),
                new InstantCommand(() -> s_Shooter.setSpeed(0), s_Shooter)
            ),
            s_Elevator.getElevatorPositionCommand(Constants.Elevator.DOWN)
        );
    }

    public Command rampCommand() {
        return new ConditionalCommand(
            this.rampVelocityPIDsToggle(0.5, this.toggle),
            this.rampVelocityPIDsToggle(0.0, this.toggle),
            () -> {return this.toggle;}
        );
    }


    public Command rampVelocityPIDsToggle(double pctspeed, boolean toggle) {
        return new FunctionalCommand(
            () -> {System.out.println("Ramping Shooter"); this.toggle = !this.toggle;},
            () -> this.s_Shooter.setSpeed(pctspeed),
            interrupted -> {},
            () -> this.s_Shooter.isFinishedRamping(pctspeed), 
            s_Shooter
        );
    }

    public Command getShootCommand() {
        return new SequentialCommandGroup(
            // this.s_Elevator.getElevatorPositionCommand(-120),
            //new InstantCommand(() -> this.s_Elevator.setElevatorSpeed(Constants.Elevator.SPEED * 1.5), this.s_Elevator),
            //this.s_Shooter.rampVelocityPIDsDifferentSpeeds(0.15, 0.15), // bottom top,
            //this.s_Shooter.rampVelocityPIDs(0.5),
            //new WaitCommand(1.0),
            new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(Constants.Intake.SPEED), s_Intake),
            new WaitCommand(.7),
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Lights.setState(Constants.Lights.LightStates.PURPLE), s_Lights),
                new InstantCommand(() -> this.s_Elevator.setElevatorSpeed(0), this.s_Elevator),
                new InstantCommand(() -> s_Intake.setDriveIntakeSpeed(0.0), s_Intake),
                new InstantCommand(() -> s_Shooter.setSpeed(0), s_Shooter)
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
