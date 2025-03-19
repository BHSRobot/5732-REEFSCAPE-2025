// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.AimWithLimelight;
//import frc.robot.Commands.Autos;
//import frc.robot.commands.ScoringPositions;
import frc.robot.subsystems.Swerve.DriveSubsystem;

import frc.robot.subsystems.Intake.Intake;
import frc.robot.utils.LimeHelp;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.Constants.MechConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
  // Robot Subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final static Intake m_Intake = new Intake();
  // Controllers
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandPS5Controller m_altdriverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);
  CommandXboxController m_OpController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  //private Autos auto;

  private final LoggedDashboardChooser<Command> autoChooser;

  //private Autos auto;

  public RobotContainer() {
    configureBindings();
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)
              ,
              true, false),

          m_robotDrive));

      configureNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    //auto = new Autos();
    new EventTrigger("Run Eject").onTrue(Commands.print("Eject Ran"));
  }

  


  private void configureBindings() {
           
    
    m_OpController.rightTrigger().
      onTrue(m_Intake.ejectCommand())
      .onFalse(m_Intake.disabledCommand());

    m_driverController.a().onTrue(
      new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive) );
    
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("score L1",  m_Intake.ejectCommand());
  }

  

  public Command getAutonomousCommand() {

    try{
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile("ScoreMid");

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
  }

    /*return new RunCommand(() -> m_robotDrive.drive(
        0.25,
        0,
        0,
        false,
        false
    ), m_robotDrive).withTimeout(1.5).andThen(new InstantCommand(() -> m_robotDrive.drive(0,
       0,
       0,
       false,
       false)));*/
    //return Commands.print("No autonomous command configured");

  }



  public void setupDriverTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addDouble("Time Remaining", () -> {
        return Timer.getMatchTime();
      }
    );
    driverTab.addString("Event Name",  () -> { 
        return DriverStation.getEventName();
      }
    );
    driverTab.addString("Alliance Color",  () -> { 
        return DriverStation.getAlliance().toString();
      }
    );
    
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
  }

  public void disablePIDSystems() {

  }

}

