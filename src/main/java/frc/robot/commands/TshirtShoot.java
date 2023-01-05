// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TshirtShoot extends SequentialCommandGroup {
  /** Creates a new TshirtShoot. */
  public TshirtShoot(DriveSubsystem m_robotDrive, Shooter m_shooter) {    
      
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
          new AutoShooterUp(m_shooter),
          new WaitCommand(1),
          new RunCommand(()->{
            m_robotDrive.tankDriveVolts(7, 7);;
          }).withTimeout(1.65),
          new InstantCommand(()->{
            m_robotDrive.tankDriveVolts(0, 0);;
          })
    );
  }
}
