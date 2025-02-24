package frc.robot.subsystems.VortexPID;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.Constants.MechConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vortex extends SubsystemBase {
    private ElevatorFeedforward m_elevatorFeedForward;
    private SparkMax m_vortexOne;
    
    private DutyCycleEncoder m_vortexEncoder;
    private ProfiledPIDController m_vortexController;

    public vortex() {
    m_vortexOne = new SparkMax(7, MotorType.kBrushless);
        
        m_vortexEncoder = new DutyCycleEncoder(9, 360,0);
        m_vortexController = new ProfiledPIDController (
            0.001,
            0,
            0,
            new TrapezoidProfile.Constraints(5, 10)
        );    
    }
    
    public void useOutput(double setpoint) {
      // TODO Auto-generated method stub
      
        m_vortexOne.set(m_vortexController.calculate(getMeasurement(), setpoint));
        System.out.println(m_vortexController.calculate(getMeasurement(), setpoint));

      
    }

    public void stop() {
        m_vortexOne.stopMotor();
    }
    
    public double getMeasurement() {
      return m_vortexEncoder.get() * (2 * Math.PI) * (180 * Math.PI);

    }
}
