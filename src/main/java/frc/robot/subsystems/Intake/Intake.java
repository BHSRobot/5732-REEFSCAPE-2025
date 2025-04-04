package frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.utils.Constants.MechConstants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.MechConstants;
import frc.robot.utils.Configs.IntakeConfigs;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    private IntakeState iState = IntakeState.DISABLED;
    private SparkMax intakeNeo;

    public Intake() {
        intakeNeo = new SparkMax(MechConstants.kIntakeID, MotorType.kBrushless);
        intakeNeo.configure(IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public enum IntakeState {
        DISABLED,
        EJECTING
    }

    public void periodic() {
      
      switch (iState) {
          case DISABLED:
              intakeNeo.set(0);
              break;
          case EJECTING:
              intakeNeo.set(.3);
              break;
        }
    }

    public void setIntakeState(IntakeState state) {
        iState = state;
    }

    public Command disabledCommand() {
        return runEnd(() -> {
            setIntakeState(IntakeState.DISABLED);
        }, () -> {});
    }
    
    public Command ejectCommand() {
        return runEnd(() -> {
            setIntakeState(IntakeState.EJECTING);
        }, () -> {});
    }
}
