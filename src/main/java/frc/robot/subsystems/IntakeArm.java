// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {
  public enum ArmState {OPENED, CLOSED};
  private static double kDt = 0.02;

  private CANSparkMax m_intakeArm = new CANSparkMax(CAN_IDs.intakeArm_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private final int m_pidSlot = 0;
  private ArmState m_armState = ArmState.CLOSED;
  private TrapezoidProfile.Constraints m_constraints = 
    new TrapezoidProfile.Constraints(IntakeArmConstants.kMaxVelocityRadPerSecond, 
              IntakeArmConstants.kMaxAccelerationRadPerSecSquared);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(IntakeArmConstants.kInitialPositionRad, 0);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
    IntakeArmConstants.kSVolts, IntakeArmConstants.kGVolts,
    IntakeArmConstants.kVVoltSecondPerRad, IntakeArmConstants.kAVoltSecondSquaredPerRad);
  private final Supplier<Double> m_leftTriggerAxisSupplier;
  private final Supplier<Double> m_rightTriggerAxisSupplier;
  
  /** Creates a new IntakeArm. */
  public IntakeArm(Supplier<Double> leftTriggerSupplier,
      Supplier<Double> rightTriggerSupplier) {
    m_leftTriggerAxisSupplier = leftTriggerSupplier;
    m_rightTriggerAxisSupplier = rightTriggerSupplier;
    m_intakeArm.restoreFactoryDefaults();
    m_intakeArm.setIdleMode(IdleMode.kBrake);
    m_intakeArm.setSmartCurrentLimit(60);

    // Verify that the soft limit is correct after experimentation 
    // m_intakeArm.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_intakeArm.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // m_intakeArm.setSoftLimit(SoftLimitDirection.kForward, IntakeArmConstants.kForwardSoftlimit);
    // m_intakeArm.setSoftLimit(SoftLimitDirection.kReverse, IntakeArmConstants.kReverseSoftLimit);

    // Set the PID coefficients for the Intake Arm motor
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_intakeArm.getPIDController();
    m_pidController.setP(IntakeArmConstants.kP, m_pidSlot);
    m_pidController.setI(IntakeArmConstants.kI, m_pidSlot);
    m_pidController.setD(IntakeArmConstants.kD, m_pidSlot);
    m_pidController.setIZone(0, m_pidSlot);
    m_pidController.setFF(0, m_pidSlot);
    m_pidController.setOutputRange(IntakeArmConstants.kMinOutput, IntakeArmConstants.kMaxOutput, m_pidSlot);

  }

  public void openArm() {
    // If the arm already opened, nothing more to do.
    if (m_armState == ArmState.OPENED) {
      return;
    } else {
      m_goal = new TrapezoidProfile.State(IntakeArmConstants.kOpenedPosArmRad, 0);
      m_armState = ArmState.OPENED;
    }
  }

  public void closeArm() {
    // If the arm already opened, nothing more to do.
    if (m_armState == ArmState.CLOSED) {
      return;
    } else {
      m_goal = new TrapezoidProfile.State(IntakeArmConstants.kClosedPosArmRad, 0);
      m_armState = ArmState.CLOSED;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_rightTriggerAxisSupplier.get() > 0) {
      closeArm();
    } else if (m_leftTriggerAxisSupplier.get() > 0) {
      openArm();
    } else {
      return;
    }

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    double feedforward = m_feedforward.calculate(m_setpoint.position, m_setpoint.velocity);
    m_pidController.setReference(m_setpoint.position * IntakeArmConstants.kArmDegToNeoRotConversionFactor, 
      ControlType.kPosition, m_pidSlot, feedforward, ArbFFUnits.kVoltage);
  }
}
