package frc.robot.subsystems.Elevator;
import frc.robot.utils.Constants.MechConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorExtension extends SubsystemBase {
    private ElevatorFeedforward m_elevatorFeedForward;
    private SparkMax m_elevOne;
    private SparkMax m_elevTwo;
    private AbsoluteEncoder m_elevEncoder;
    private ProfiledPIDController m_elevPid;

    public ElevatorExtension() {
        //m_elevOne = new SparkMax(MechConstants.kElevLeadID, MotorType.kBrushless);
        //m_elevTwo = new SparkMax(MechConstants.kElevTwoID, MotorType.kBrushless);
        
        m_elevPid = new ProfiledPIDController (
                    MechConstants.kPElevLen,
                    MechConstants.kIElevLen,
                    MechConstants.kDElevLen,
                    new TrapezoidProfile.Constraints(MechConstants.kElevMaxVel, MechConstants.kElevMaxAccel)
                );

        
        
    }


    
    

    


    
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
      // TODO Auto-generated method stub
      // Calculate the feedforward from the sepoint
      double feedforwardval = m_elevatorFeedForward.calculate(setpoint.position, setpoint.velocity);
      // Add the feedforward to the PID output to get the motor output
      if (getMeasurement() >= 1.25) {
        m_elevOne.set(0);
        
      }
      
      else {
        m_elevOne.set(output);
        
      }
      
    }

    
    public double getMeasurement() {
      return m_elevEncoder.getPosition();
    }
    
}
