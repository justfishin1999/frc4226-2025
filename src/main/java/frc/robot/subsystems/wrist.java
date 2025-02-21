package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class wrist extends SubsystemBase {
    SparkFlex m_wristMotor;
    SparkFlexConfig m_motorConfig;
    SparkClosedLoopController m_closedLoopController;
    RelativeEncoder m_encoder;
  
    /** Creates a new Climber. */
    public wrist() {
      m_wristMotor = new SparkFlex(Constants.wristConstants.wristMotorID, MotorType.kBrushless);
  
      m_motorConfig = new SparkFlexConfig();
  
      m_closedLoopController = m_wristMotor.getClosedLoopController();
      m_encoder = m_wristMotor.getEncoder();
      
      m_motorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .p(Constants.wristConstants.kP)
          .i(Constants.wristConstants.kI)
          .d(Constants.wristConstants.kD)
          .velocityFF(Constants.wristConstants.kFF)
          .outputRange(Constants.minMaxOutputConstants.kMinOutput, Constants.minMaxOutputConstants.kMaxOutput, ClosedLoopSlot.kSlot1);
  
      m_motorConfig.closedLoop.maxMotion
          // Set MAXMotion parameters for position control. We don't need to pass
          // a closed loop slot, as it will default to slot 0.
          .maxVelocity(Constants.wristConstants.kMaxMotionVelocity)
          .maxAcceleration(Constants.wristConstants.kMaxMotionAcceleration)
          .allowedClosedLoopError(1);
      try{
        m_wristMotor.configure(m_motorConfig,ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        System.out.println("!!Successfully configured wrist motor!!");
      }
      catch (Exception e1){
        System.err.println("Failed to apply wrist motor configurations: "+e1.toString());
        DriverStation.reportWarning("Failed to apply wrist motor configuration",true);
      }
      
  
    }
  
    @Override
    public void periodic() {
      SmartDashboard.putNumber("wrist Velocity",m_encoder.getVelocity());
      SmartDashboard.putNumber("wrist Position",m_encoder.getPosition());
      // This method will be called once per scheduler run
    }
  
    public void actuateWrist(double setpoint){
      m_closedLoopController.setReference(setpoint,ControlType.kVelocity);
    }
  
    public void resetEncoder(){
      m_encoder.setPosition(0.0);
    }
  
    public void stop(){
      m_closedLoopController.setReference(0,ControlType.kVelocity);
    }
  }