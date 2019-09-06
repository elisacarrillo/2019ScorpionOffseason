package frc.robot;

public class Constants {

	public static boolean isCompetitionBot = true;

	public static double kTurret_Motion_turnP = 0.75;
	public static double kTurret_Motion_turnI = 0.0;
	public static double kTurret_Motion_turnD = 0.0;
	public static double kD_Motion_turnIZone = 5;

	public static double kDrive_Motion_turnP = 0.75;
	public static double kDrive_Motion_turnI = 0.5;
	public static double kDrive_Motion_turnD = 0.2;
	//public static double kD_Motion_turnIZone = 5;
	//ifcompbot
	public static boolean compbot = true;
	//feed ARM PID
	public static double intakeArmP = 0;
	public static double intakeArmI = 0.1;
	public static double intakeArmD = 0.1;
	public static double intakeArmF = 0;
	public static double intakeArmIZone = 0;

	public final static double armLength = 16.5;

	public static double kLift_Tolerance = 2.0;
	public static double kLift_NudgeAmount = 50.0;
	public static double kShoulder_Tolerance = 7.0; //degrees
	public static double kWrist_Tolerance = 7.0; //degrees

	public static double kElevatorShoulderWrist_SetpointWait = 0.1;

	public static double normalPipeline = 0;
	public static double snakeLimePipeline = 7;

	//Motion Profiling Constants
															// units
	public static double kDrive_Motion_P = 1.4;				// %/ft
	public static double kDrive_Motion_D = 0.0;				// %/(ft/s)
	public static double kDrive_Motion_V = 0.058;			// %/(ft/s) max turn speed
	public static double kDrive_Motion_A = 0.0;	
	public static double kDrive_peakOutput = 0.8;
	public static double kDrive_ContinuousCurrentLimit = 60;
	public static double kDrive_PeakCurrentLimit = 80;
	public static double kDrive_PeakCurrentTime_ms = 100;
	public static double kDrive_Motion_trackwidth = 2.72;
	public static double kDrive_WheelDiameterInch = 6.0;
	public static double kDrive_Motion_Velocity = 6.0;
	public static double kDrive_Motion_Acceleration = 13.0;
	public static double getWheelCircumference() { 
		return (kDrive_WheelDiameterInch*Math.PI)/12.0; 
	};
}