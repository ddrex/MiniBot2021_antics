package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
//import frc.robot.subsystems.Servo2;
import frc.robot.subsystems.Servo3;

public class GoUp extends CommandBase {
  private Servo3 m_arm;

  private double m_delta = 0.005;

  /** Creates a new RaiseArm. */
  public GoUp(Servo3 servo) {
   m_arm = servo;
    addRequirements(m_arm);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    m_arm.Up();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
