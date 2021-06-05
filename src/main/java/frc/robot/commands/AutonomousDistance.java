// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Alisha Paul

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Servo3;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.commands.ServoUp;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param Servo The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Servo3 servo, Drivetrain drivetrain) {
    addCommands(

       new DriveDistance(0.6, 0.3, drivetrain),
        new GoUp(servo),
        new WaitCommand(2),
        new GoDown(servo));
        
        /*new TurnDegrees(-0.6, 1.2, drivetrain),//1.75 1.55
        new DriveDistance(0.6, 0.27, drivetrain),
       // Who's a good Robot, You are!!
        new TurnDegrees(-0.6, 0.52, drivetrain),//0.47
        new DriveDistance(-0.6, 0.675, drivetrain),//0.725
        new TurnDegrees(-0.6, 1.3, drivetrain),//0.99 1.1 0.9 1.05
        new DriveDistance(-0.6, 0.3, drivetrain),//0.25
        new TurnDegrees(-0.6, 1.425, drivetrain),//1.405
        new DriveDistance(-0.6, 0.7, drivetrain),//0.6
       // You're doing great!!
        //new TurnDegrees(-0.6, 0.8, drivetrain),//1
        new DriveDistance(0.6, 0.65, drivetrain),//0.6
        new TurnDegrees(-0.6, 1.5, drivetrain),//1.3 1.65
        new DriveDistance(0.6, 0.6, drivetrain), //0.45
        new TurnDegrees(-0.6, 1.65, drivetrain),//0.9
        new DriveDistance(0.6, 0.675, drivetrain),//0.6
        
        new DriveDistance(-0.6, 0.25, drivetrain),
        new TurnDegrees(-0.6, 1.65, drivetrain),
        new DriveDistance(-0.6, 0.33, drivetrain));//0.4*/

//Not super batteries
/*new DriveDistance(0.6, 0.3, drivetrain),
        new TurnDegrees(-0.6, 1.65, drivetrain),//1.35
        new DriveDistance(0.6, 0.27, drivetrain),
       
        new TurnDegrees(-0.6, 0.47, drivetrain),//1.23
        new DriveDistance(-0.6, 0.725, drivetrain),//0.75
        new TurnDegrees(-0.6, 1.1, drivetrain),//0.99
        new DriveDistance(-0.6, 0.3, drivetrain),//0.25
        new TurnDegrees(-0.6, 1.45, drivetrain),//1.85 0.9
        new DriveDistance(-0.6, 0.60, drivetrain),
       
        //new TurnDegrees(-0.6, 0.8, drivetrain),//1
        new DriveDistance(0.6, 0.58, drivetrain),//0.7
        new TurnDegrees(-0.6, 1.65, drivetrain),//1.3
        new DriveDistance(0.6, 0.6, drivetrain), //0.45
        new TurnDegrees(-0.6, 1.5, drivetrain),//0.9
        new DriveDistance(0.6, 0.50, drivetrain),
        
        new DriveDistance(-0.6, 0.25, drivetrain),
        new TurnDegrees(-0.6, 1.65, drivetrain),
        new DriveDistance(-0.6, 0.4, drivetrain));//0.55
*/
        
        //Good auto, slightly used batteries
/* new DriveDistance(0.6, 0.3, drivetrain),
        new TurnDegrees(-0.6, 1.35, drivetrain),
        new DriveDistance(0.8, 0.23, drivetrain),
       
        new TurnDegrees(-0.6, 1.2, drivetrain),//1.15
        new DriveDistance(-0.6, 0.75, drivetrain),//0.65
        new TurnDegrees(-0.6, 1.1, drivetrain),//0.99
        new DriveDistance(-0.6, 0.2, drivetrain),
        new TurnDegrees(-0.6, 1.85, drivetrain),//2.1
        new DriveDistance(-0.8, 0.60, drivetrain),
       
        //new TurnDegrees(-0.6, 0.8, drivetrain),//1
        new DriveDistance(0.6, 0.70, drivetrain),
        new TurnDegrees(-0.6, 1.3, drivetrain),
        new DriveDistance(0.6, 0.45, drivetrain), 
        new TurnDegrees(-0.6, 0.9, drivetrain),//0.9
        new DriveDistance(0.8, 0.50, drivetrain),
        
        new DriveDistance(-0.6, 0.25, drivetrain),
        new TurnDegrees(-0.6, 1.65, drivetrain),
        new DriveDistance(-0.6, 0.4, drivetrain));//0.55*/

  }
}
