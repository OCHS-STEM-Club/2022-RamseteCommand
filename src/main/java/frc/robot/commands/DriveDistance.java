// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.XboxController;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;
  /** Creates a new DriveDistance. */
  public DriveDistance(double inches, double speed, DriveSubsystem drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
    m_drive.tankDriveVolts(4, 4);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.tankDriveVolts(4, 4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getAverageEncoderDistance()) >= m_distance;
    //return false;
  }
}
