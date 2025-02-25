// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.celestial;

import java.util.HashMap;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.celestial.Constants.OperatorConstants;
import com.celestial.commands.SwerveJoystickCommand;
import com.celestial.subsystems.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController commandController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final XboxController controller = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    
    PathPlannerPath path = PathPlannerPath.fromPathFile("straight");
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
                swerveSubsystem,
                () -> controller.getRawAxis(Constants.OIConstants.kDriverYAxis),
                () -> controller.getRawAxis(Constants.OIConstants.kDriverXAxis),
                () -> controller.getRawAxis(Constants.OIConstants.kDriverRotAxis),
                () -> !controller.getLeftBumperButton()));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        HashMap<String, Command> eventMap = new HashMap<>();
        return new SequentialCommandGroup(
              new InstantCommand(() -> {
                      if(true){
                              swerveSubsystem.resetOdometry(path.getStartingHolonomicPose());
                      }
              }),
              new FollowPathCommand(
                      path,
                      swerveSubsystem.getPose(),
                      speedsSupplier,
                      output,
                      new PPHolonomicDriveController(new PIDController(0, 0, 0), new PIDController(0, 0, 0))
                      robotconfig,
                      false,
                      swerveSubsystem
              )
      );
    }
}
