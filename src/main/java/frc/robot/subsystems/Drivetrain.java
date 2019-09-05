/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import frc.robot.ninjaLib.MotionUtils;
import frc.robot.ninjaLib.Values;
import frc.robot.RobotMap;

import  static frc.robot.HAL.navX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.HAL;
import frc.robot.OI;

import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.commands.DriveCommand;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DifferentialDrive drive;

  public final WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.LEFT_MASTER_PORT);
  public final CANSparkMax leftFollowerA = new CANSparkMax(RobotMap.LEFT_SLAVE_PORT_1, MotorType.kBrushless);
  public final CANSparkMax leftFollowerB = new CANSparkMax(RobotMap.LEFT_SLAVE_PORT_2, MotorType.kBrushless);

  public final WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.RIGHT_MASTER_PORT);
  public final CANSparkMax rightFollowerA = new CANSparkMax(RobotMap.RIGHT_SLAVE_PORT_1, MotorType.kBrushless);
  public final CANSparkMax rightFollowerB = new CANSparkMax(RobotMap.RIGHT_SLAVE_PORT_2, MotorType.kBrushless);

  CANEncoder leftEncoderFollowerA = new CANEncoder(leftFollowerA);
  CANEncoder leftEncoderFollowerB = new CANEncoder(leftFollowerB);
  CANEncoder rightEncoderFollowerA = new CANEncoder(rightFollowerA);
  CANEncoder rightEncoderFollowerB = new CANEncoder(rightFollowerB);

  public CANPIDController leftPID = new CANPIDController(leftFollowerA);
  public CANPIDController leftPID2 = new CANPIDController(leftFollowerB);

  public CANPIDController rightPID = new CANPIDController(rightFollowerA);
  public CANPIDController rightPID2 = new CANPIDController(rightFollowerB);

  public Drivetrain (){
    rightFollowerA.set(rightMaster.getMotorOutputPercent());
    rightFollowerB.set(rightMaster.getMotorOutputPercent());
    leftFollowerA.set(rightMaster.getMotorOutputPercent());
    leftFollowerB.set(rightMaster.getMotorOutputPercent());

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.enableVoltageCompensation(true);
    leftMaster.configVoltageCompSaturation(12, 10);

    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.enableVoltageCompensation(true);
    rightMaster.configVoltageCompSaturation(12, 10);

    leftMaster.configForwardSoftLimitEnable(false, 10);
    leftMaster.configReverseSoftLimitEnable(false, 10);
    leftMaster.setControlFramePeriod(ControlFrame.Control_3_General, 5);
    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);
    leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms, 10);

    rightMaster.configForwardSoftLimitEnable(false, 10);
    rightMaster.configReverseSoftLimitEnable(false, 10);
    rightMaster.setControlFramePeriod(ControlFrame.Control_3_General, 5);
    rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);
    rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms, 10);

    configMotorControllers(10);
    
    setBrake(true);
    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);
    drive.setMaxOutput(1.0);
  }

  public void configMotorControllers(int timeout){
    double kWheelCircumference = Constants.getWheelCircumference();  
    leftMaster.setSensorPhase(false);
		leftMaster.setInverted(false); // change here
		leftFollowerA.setInverted(false);
		leftFollowerB.setInverted(false);
		//leftFollowerC.setInverted(false);

    leftMaster.configSetParameter(ParamEnum.eContinuousCurrentLimitAmps, Constants.kDrive_ContinuousCurrentLimit, 0x00, 0x00, timeout);
		leftMaster.configSetParameter(ParamEnum.ePeakCurrentLimitAmps, Constants.kDrive_PeakCurrentLimit, 0x00, 0x00, timeout);
		leftMaster.configSetParameter(ParamEnum.ePeakCurrentLimitMs, Constants.kDrive_PeakCurrentTime_ms, 0x00, 0x00, timeout);
		leftMaster.enableCurrentLimit(false);
		
		leftMaster.configPeakOutputForward(Constants.kDrive_peakOutput, timeout);
		leftMaster.configPeakOutputReverse(-Constants.kDrive_peakOutput, timeout);
    leftMaster.configOpenloopRamp(0.0, timeout);
    
    rightMaster.setSensorPhase(false);
		rightMaster.setInverted(false);
		rightFollowerA.setInverted(false);
		rightFollowerB.setInverted(false);
  //	rightFollowerC.setInverted(false);
  
    rightMaster.configSetParameter(ParamEnum.eContinuousCurrentLimitAmps, Constants.kDrive_ContinuousCurrentLimit, 0x00, 0x00, timeout);
		rightMaster.configSetParameter(ParamEnum.ePeakCurrentLimitAmps, Constants.kDrive_PeakCurrentLimit, 0x00, 0x00, timeout);
		rightMaster.configSetParameter(ParamEnum.ePeakCurrentLimitMs, Constants.kDrive_PeakCurrentTime_ms, 0x00, 0x00, timeout);
		rightMaster.enableCurrentLimit(false);
		
		rightMaster.configPeakOutputForward( Constants.kDrive_peakOutput, timeout);
		rightMaster.configPeakOutputReverse(-Constants.kDrive_peakOutput, timeout);
    rightMaster.configOpenloopRamp(0.0, timeout);
    
    int nativeVelocity = (int) (Constants.kDrive_Motion_Velocity * 1.0/Constants.getWheelCircumference() / 10.0);
    int nativeAcceleration = (int) (Constants.kDrive_Motion_Acceleration * 1.0/Constants.getWheelCircumference() / 10.0);

    leftMaster.configMotionCruiseVelocity(nativeVelocity, timeout);
		leftMaster.configMotionAcceleration(nativeAcceleration, timeout);
		rightMaster.configMotionCruiseVelocity(nativeVelocity, timeout);
		rightMaster.configMotionAcceleration(nativeAcceleration, timeout);


    leftPID.setP(1);
    leftPID.setI(0);
    leftPID.setD(0);
    leftPID2.setP(1);
    leftPID2.setI(0);
    leftPID2.setD(0);

    rightPID.setP(1);
    rightPID.setI(0);
    rightPID.setD(0);
    
  }

  public void arcade(double move, double turn){
    drive.arcadeDrive(move, turn);
  }

  public void tank(double left, double right)
  {
    drive.tankDrive(left, right);
  }

  
	public void resetEncoders() {
		//leftMaster.setSelectedSensorPosition(0, 0, 0);
		//rightMaster.setSelectedSensorPosition(0, 0, 0);
  }

  public double getLeftPosition() {
		return MotionUtils.rotationsToDistance(leftEncoderFollowerA.getPosition(), Constants.getWheelCircumference());
  }
  
  public double getLeftPositionTalon() {
    return MotionUtils.rotationsToDistance(MotionUtils.ticksToRotations(leftMaster.getSelectedSensorPosition(), 4096, 1), Constants.getWheelCircumference());
  }
	
	public double getRightPosition() {
		return MotionUtils.rotationsToDistance(rightEncoderFollowerA.getPosition(), Constants.getWheelCircumference());
  }

  public double getRightPositionTalon() {
    return MotionUtils.rotationsToDistance(MotionUtils.ticksToRotations(leftMaster.getSelectedSensorPosition(), 4096, 1), Constants.getWheelCircumference());
  }


  public double getLeftVelocity() {
		return leftEncoderFollowerA.getVelocity() /  10.0 * Constants.getWheelCircumference();
	}
	
	public double getRightVelocity() {
		return rightEncoderFollowerA.getVelocity() /  10.0 * Constants.getWheelCircumference();
  }
  
  public void positionPDauxF(double leftPos, double leftFF, double rightPos, double rightFF) {
		leftMaster.selectProfileSlot(1, 0);
		rightMaster.selectProfileSlot(1, 0);

		leftMaster.set(ControlMode.Position, MotionUtils.distanceToRotations(leftPos, Constants.getWheelCircumference()) * 4096, DemandType.ArbitraryFeedForward, leftFF);
		rightMaster.set(ControlMode.Position, MotionUtils.distanceToRotations(rightPos, Constants.getWheelCircumference()) * 4096, DemandType.ArbitraryFeedForward, -rightFF);
  }
	
  
  public void setBrake(boolean brake) {
    NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    if(brake) {
      leftMaster.setNeutralMode(mode);
      leftFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
      leftFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rightMaster.setNeutralMode(mode);
      rightFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rightFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);
    } else {
      leftMaster.setNeutralMode(mode);
      leftFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
      leftFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rightMaster.setNeutralMode(mode);
      rightFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rightFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
  }

 // public void moveFoward(int speed, int timeout){
 //  rightMaster.set(speed);
 //    rightMaster.setCANTimeout(timeout);
 // }

  public void followLimeLight()
  {
      leftMaster.set(-HAL.limelight.xOffset()/108);
      rightMaster.set(-HAL.limelight.xOffset()/108);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveCommand(OI.throttle,OI.turn));
    
  }
}
