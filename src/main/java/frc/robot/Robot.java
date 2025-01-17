// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/* 
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.s_Elevator.setFlapAngles(Constants.Elevator.RIGHT_FLAP_IN, Constants.Elevator.LEFT_FLAP_IN);
    m_robotContainer.s_Shooter.setSpeed(0.0);
    m_robotContainer.s_Intake.getPosAndRunIntakeCommand(Constants.Intake.UP_POSITION, 0.0).schedule();
    //m_robotContainer.s_Elevator.getElevatorPositionCommand(Constants.Elevator.DOWN).schedule();
    //m_robotContainer.s_Climber.setPosition(Constants.Climber.DOWN_POSITION);

    //m_robotContainer.s_Elevator.setElevatorPosition(48);
   // m_robotContainer.s_Lights.setState(LightStates.PURPLE);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //m_robotContainer.s_Intake.setPosition(Constants.Intake.UP_POSITION).schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_robotContainer.s_Elevator.setFlapAngles(60, 0);
    // System.out.println(m_robotContainer.s_Intake.getSensor());
 // m_robotContainer.s_Intake.setDriveIntakeSpeed(0.50);
    //m_robotContainer.s_Elevator.SmartDashboard();
   // m_robotContainer.s_Elevator.setElevatorSpeed(0.4);
    /*
    if(controller.getBButton()) {
      m_robotContainer.s_Shooter.shootWhenClose(m_robotContainer.s_Limelight, .3);
    } else {
      m_robotContainer.s_Shooter.setSpeed(0.0);
    }
    */
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }  

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}