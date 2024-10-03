// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer
{

  final CommandXboxController m_driveController = new CommandXboxController(1);
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(1,2,3);

  private final PathPlannerPath m_10ft_forward = PathPlannerPath.fromPathFile("Straight Line");   
  private final PathPlannerPath m_curve = PathPlannerPath.fromPathFile("Curve");     
  private final PathPlannerPath m_try = PathPlannerPath.fromPathFile("Try");                                                         
                                                           

  public SendableChooser<Command> autoChooser;
  public RobotContainer()
  {
    configureBindings();
    
    Command driveFieldOrientedAngularVelocity = m_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(m_driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

    autoChooser = new SendableChooser<Command>();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.setDefaultOption("10ft forward", AutoBuilder.followPath(m_10ft_forward));
    autoChooser.addOption("Curve", AutoBuilder.followPath(m_curve));
    autoChooser.addOption("Try", AutoBuilder.followPath(m_try));
  }

  private void configureBindings()
  {

    m_driveController.a().onTrue((Commands.runOnce(m_SwerveSubsystem::zeroGyro)));
    m_driveController.x().whileTrue(Commands.runOnce(m_SwerveSubsystem::lock, m_SwerveSubsystem).repeatedly());
    m_driveController.rightTrigger(0.1).whileTrue(Commands.runOnce(m_IntakeSubsystem::intakeIn, m_IntakeSubsystem).repeatedly());
    m_driveController.leftTrigger(0.1).whileTrue(Commands.runOnce(m_IntakeSubsystem::intakeOut, m_IntakeSubsystem).repeatedly());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void setMotorBrake(boolean brake)
  {
    m_SwerveSubsystem.setMotorBrake(brake);
  }

  public Command getAutonomousCommand()
  {
    m_SwerveSubsystem.resetOdometry(m_SwerveSubsystem.getPose());
    m_SwerveSubsystem.zeroGyro();
    return autoChooser.getSelected();
  }

  
}