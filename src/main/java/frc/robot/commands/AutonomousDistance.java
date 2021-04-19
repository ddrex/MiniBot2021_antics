// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Alisha Paul

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(0.6, 0.3, drivetrain),
        new TurnDegrees(-0.6, 1.35, drivetrain),
        new DriveDistance(0.8, 0.23, drivetrain),
       
        new TurnDegrees(-0.6, 1.15, drivetrain),
        new DriveDistance(-0.6, 0.65, drivetrain),
        new TurnDegrees(-0.6, 0.99, drivetrain),
        new DriveDistance(-0.6, 0.2, drivetrain),
        new TurnDegrees(-0.6, 2.1, drivetrain),
        new DriveDistance(-0.8, 0.60, drivetrain),
       
        new TurnDegrees(-0.6, 1.0, drivetrain),
        new DriveDistance(0.6, 0.70, drivetrain),
        new TurnDegrees(-0.6, 1.3, drivetrain),
        new DriveDistance(0.6, 0.45, drivetrain), 
        new TurnDegrees(-0.6, 0.9, drivetrain),
        new DriveDistance(0.8, 0.50, drivetrain),
        
        new DriveDistance(-0.6, 0.25, drivetrain),
        new TurnDegrees(-0.6, 1.65, drivetrain),
        new DriveDistance(-0.6, 0.55, drivetrain));




  }
}
