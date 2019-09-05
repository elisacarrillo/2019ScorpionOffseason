package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;
  //
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  //OI
  public static final int JOYSTICK_PORT = 0;

  //CHANNELS
  public final static int ULTRA_PING_CHANNEL = 1;
  public final static int ULTRA_ECHO_CHANNEL = 1;

  //DRIVE
  public static final int RIGHT_MASTER_PORT = 20;
  public static final int RIGHT_SLAVE_PORT_1 = 1;
  public static final int RIGHT_SLAVE_PORT_2 = 2;
  public static final int LEFT_MASTER_PORT = 21;
  public static final int LEFT_SLAVE_PORT_1 = 3;
  public static final int LEFT_SLAVE_PORT_2 = 4;

  //CLIMB
  public final static int TURTLE_MASTER_PORT = 4;
  public final static int TURTLE_SLAVE_PORT = 5;
  
  public final static int INTAKE_ARM_PORT = 6;
  public final static int HATCH_GROUND_INTAKE_PORT = 7;
  
  //ELEVATOR
  public final static int ELEVATOR_MASTER_PORT = 8;
  public final static int ELEVATOR_SLAVE_PORT = 9;

  //TURRET
  public final static int TURRET_PORT = 10;

  //MECHANISM
  public final static int SHOULDER_PORT = 11;
  public final static int WRIST_PORT = 12;
  public final static int FEEDER_BALL_PORT = 13;

  //VISION STATUS
  public static boolean vision = false;
  

  

}
