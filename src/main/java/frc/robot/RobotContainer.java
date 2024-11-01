// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer
{

  final CommandXboxController m_driveController = new CommandXboxController(1);
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public RobotContainer()
  {
    configureBindings();
    
    Command driveFieldOrientedAngularVelocity = m_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(m_driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

        m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
}

  private void configureBindings()
  {
    m_driveController.a().onTrue((Commands.runOnce(m_SwerveSubsystem::zeroGyro)));
    m_driveController.x().whileTrue(Commands.runOnce(m_SwerveSubsystem::lock, m_SwerveSubsystem).repeatedly());
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
    try{
      return new PathPlannerAuto("Auto");
    } catch(Exception e ) {
      throw e;
    }
  }

  public void resetGyro(){
    m_SwerveSubsystem.zeroGyro();
  }
  
}