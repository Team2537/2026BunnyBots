package frc.robot.subsystems.drive.arm;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmIOReal implements ArmIO {
    /** Creates a new ArmIOReal. */
    private final TalonFX leftPivotMotor = new TalonFX(ArmConstants.LEFT_PIVOT_MOTOR_ID);
    private final TalonFX rightPivotMotor = new TalonFX(ArmConstants.RIGHT_PIVOT_MOTOR_ID);
    private final TalonFX leftGrabberMotor = new TalonFX(ArmConstants.LEFT_GRABBER_MOTOR_ID);
    private final TalonFX rightGrabberMotor = new TalonFX(ArmConstants.RIGHT_GRABBER_MOTOR_ID);

    private final StatusSignal<?> leftPivotAppliedVolts;
    private final StatusSignal<?> leftPivotPosition;
    private final StatusSignal<?> leftPivotVelocity;
    private final StatusSignal<?> leftPivotSupplyCurrent;
    private final StatusSignal<?> leftPivotStatorCurrent;

    private final StatusSignal<?> rightPivotAppliedVolts;
    private final StatusSignal<?> rightPivotPosition;
    private final StatusSignal<?> rightPivotVelocity;
    private final StatusSignal<?> rightPivotSupplyCurrent;
    private final StatusSignal<?> rightPivotStatorCurrent;





  public ArmIOReal() {
    configureLeftPivotMotor();
    configureRightPivotMotor();
    configureLeftGrabberMotor();
    configureRightGrabberMotor();
  }
  private void configureLeftPivotMotor() {
        // Configure the left pivot motor settings here
    
  }
  private void configureRightPivotMotor() {
        // Configure the right pivot motor settings here
    
  }
  private void configureLeftGrabberMotor() {
        // Configure the left grabber motor settings here
    
  }
  private void configureRightGrabberMotor() {
        // Configure the right grabber motor settings here
    
  }

}