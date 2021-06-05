// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Servo2;

// public class ServoUp extends CommandBase {
//   private final Servo2 m_arm;

//   private double m_delta = 0.005;

//   /** Creates a new RaiseArm. */
//   public ServoUp(Servo2 servo) {
//     m_arm = RobotContainer.servo;
//     addRequirements(m_arm);

//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {    
//     m_arm.incrementArm(-m_delta);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
