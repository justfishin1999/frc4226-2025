package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  SparkMax m_IntakeMotor;
  SparkMaxConfig m_motorConfig;
  SparkClosedLoopController m_closedLoopController;
  RelativeEncoder m_encoder;

  /** Creates a new Intake. */
  public Intake() {
    m_IntakeMotor = new SparkMax(Constants.IntakeConstants.IntakeMotorID, MotorType.kBrushless);

    m_motorConfig = new SparkMaxConfig();

    m_closedLoopController = m_IntakeMotor.getClosedLoopController();
    m_encoder = m_IntakeMotor.getEncoder();
    
    m_motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(Constants.IntakeConstants.kP)
        .i(Constants.IntakeConstants.kI)
        .d(Constants.IntakeConstants.kD)
        .velocityFF(Constants.IntakeConstants.kFF)
        .outputRange(Constants.minMaxOutputConstants.kMinOutput, Constants.minMaxOutputConstants.kMaxOutput, ClosedLoopSlot.kSlot1);

    m_motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(Constants.IntakeConstants.kMaxMotionVelocity)
        .maxAcceleration(Constants.IntakeConstants.kMaxMotionAcceleration)
        .allowedClosedLoopError(1);
    try{
      m_IntakeMotor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      System.out.println("!!Successfully configured Intake motor!!");
    }
    catch (Exception e1){
      System.err.println("Failed to apply Intake motor configurations: "+e1.toString());
      DriverStation.reportWarning("Failed to apply Intake motor configuration",true);
    }
    

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Velocity",m_encoder.getVelocity());
    SmartDashboard.putNumber("Intake Position",m_encoder.getPosition());
    // This method will be called once per scheduler run
  }

  public void runIntake(double velocity){
    m_closedLoopController.setReference(velocity,ControlType.kVelocity);
  }

  public void resetEncoder(){
    m_encoder.setPosition(0.0);
  }

  public void stop(){
    m_closedLoopController.setReference(0,ControlType.kVelocity);
  }
}