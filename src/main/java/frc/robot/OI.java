package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

import java.util.function.DoubleSupplier;

import frc.robot.commands.*;
import frc.robot.ninjaLib.Gamepad;
import frc.robot.subsystems.*;
import frc.robot.ninjaLib.CustomButton;
//import frc.robot.ninjaLib.AxisButton;
import frc.robot.ninjaLib.ButtonBoard;
import frc.robot.ninjaLib.ButtonBoard.Direction;
import frc.robot.HAL;
import static frc.robot.HAL.ballFeed;
import static frc.robot.HAL.hatchGroundIntake;


public class OI {
 
  /////////////////////
  // XBOX CONTROLLER //
  /////////////////////

  public static Gamepad driver = new Gamepad(0);
  public static Gamepad vision = new Gamepad(4);
  
  // DRIVE
  public static DoubleSupplier turn = () -> (driver.getRightX()*.8); //was .7
  public static DoubleSupplier throttle = () -> (-driver.getLeftY()*.95); //was .9

  // BALL INTAKE
  public static Button ballFeedIn = new JoystickButton(driver,Gamepad.BUTTON_SHOULDER_RIGHT);
  public static Button ballFeedOut = new JoystickButton(driver,Gamepad.BUTTON_SHOULDER_LEFT);

  // VISION
  public static Button highLime = new JoystickButton(driver, Gamepad.BUTTON_X);
  //public static Button shoulderDown = new JoystickButton(driver, Gamepad.BUTTON_A);

  public static Button turretLime = new JoystickButton(driver, Gamepad.BUTTON_Y);
  public static DoubleSupplier turretLime7 = () -> (driver.getRightTrigger());
  //public static Button turretLime4 = new AxisButton(driver, 1);
  public static Button switchVision = new JoystickButton(driver, Gamepad.BUTTON_B);
  

  /////////////////
  // BUTTONBOARD //
  /////////////////

  public static ButtonBoard buttonBoardA = new ButtonBoard(1);
  public static ButtonBoard buttonBoardB = new ButtonBoard(2);
      
  //////////////////////////////
  // NEW SCORPION BUTTONBOARD //
  //////////////////////////////
  //visionTurret
  public static Button hatchFeederStation = new JoystickButton(buttonBoardA, 10);
  public static Button visionTurret = new JoystickButton(buttonBoardA, 9);

  Button ballFeedGround = new JoystickButton(buttonBoardA, 1);

  Button lowBallRocketStag = new JoystickButton(buttonBoardB, 3);
  Button midBallRocketStag = new JoystickButton(buttonBoardB, 6);
  Button highBallRocketStag = new JoystickButton(buttonBoardB, 5);

  Button lowHatchRocketStag = new JoystickButton(buttonBoardB, 7);
  Button midHatchRocketStag = new JoystickButton(buttonBoardB, 11);
  Button highHatchRocketStag = new JoystickButton(buttonBoardB, 12);
  
  Button cargoshipBallStag = new JoystickButton(buttonBoardB, 8);
  Button cargoshipHatchStag = new JoystickButton(buttonBoardA, 9);
  

  Button travelPosition = new JoystickButton(buttonBoardA, 12);
  Button hatchGrab = new JoystickButton(buttonBoardA, 11);
  Button hatchRelease = new JoystickButton(buttonBoardA, 2);

  Button turretFront = new JoystickButton(buttonBoardA, 7);
  Button turretBack = new JoystickButton(buttonBoardA, 8);
  Button turretRight = new JoystickButton(buttonBoardA, 6);
  Button turretLeft = new JoystickButton(buttonBoardA, 5);
  
  // Button turretRocketRightBack = new JoystickButton(buttonBoardB, 9);

  Button frontClimbHome = new JoystickButton(buttonBoardA, 3);
  Button frontClimbPassLvl_3 = new JoystickButton(buttonBoardB, 2); //should be B2
  Button frontClimbPassLvl_2 = new JoystickButton(buttonBoardB, 9); //should be B2

  // Button frontClimbGround = new JoystickButton(buttonBoardA, 2);

  
  Button climbAutoLvl_3 = new JoystickButton(buttonBoardB, 4);
  Button climbAutoLvl_2 = new JoystickButton(buttonBoardB, 10);
  Button climbStop = new JoystickButton(buttonBoardB, 1);

  Button turretZero = new JoystickButton(buttonBoardA, 4); //should be A4
  
    Button anyTurret = new CustomButton(() -> {
      return turretFront.get() || turretBack.get() || turretRight.get() || turretLeft.get();
    });

  ///////////////////
  // BUTONBOARD V1 //
  ///////////////////

  // Button ballFeedGround = new JoystickButton(buttonBoardA, 1);
 
  // Button lowBallRocketStag = new JoystickButton(buttonBoardB, 3);
  // Button midBallRocketStag = new JoystickButton(buttonBoardB, 6);
  // Button highBallRocketStag = new JoystickButton(buttonBoardB, 5);

  // Button lowHatchRocketStag = new JoystickButton(buttonBoardB, 7);
  // Button midHatchRocketStag = new JoystickButton(buttonBoardB, 11);
  // Button highHatchRocketStag = new JoystickButton(buttonBoardB, 12);
  
  // Button cargoshipBallStag = new JoystickButton(buttonBoardB, 8);

  // Button travelPosition = new JoystickButton(buttonBoardA, 12);
  // Button hatchGrab = new JoystickButton(buttonBoardB, 10);
  // Button hatchRelease = new JoystickButton(buttonBoardA, 11);

  // Button turretFront = new JoystickButton(buttonBoardA, 7);
  // Button turretBack = new JoystickButton(buttonBoardA, 8);
  // Button turretRight = new JoystickButton(buttonBoardA, 6);
  // Button turretLeft = new JoystickButton(buttonBoardA, 5);
  
  // Button turretRocketRightBack = new JoystickButton(buttonBoardB, 9);

  // Button frontClimbHome = new JoystickButton(buttonBoardA, 3);
  // Button frontClimbPass = new JoystickButton(buttonBoardB, 2); //should be B2
  // Button frontClimbGround = new JoystickButton(buttonBoardA, 2);

  
  // Button climbAuto = new JoystickButton(buttonBoardB, 4);
  // Button climbStop = new JoystickButton(buttonBoardB, 1);

  // Button transfer = new JoystickButton(buttonBoardA, 4); //should be A4

  // Button anyTurret = new CustomButton(() -> {
  //   return turretFront.get() || turretBack.get() || turretRight.get() || turretLeft.get();
  // });


  ////////////////////////////
  // TESTING MANUAL CONTROL //
  ////////////////////////////
  public static DoubleSupplier turret = () -> -buttonBoardA.getX()/8;  
  public static DoubleSupplier elevator = () -> buttonBoardA.getY();

  public static Joystick turtleFront = new Joystick(4);
  public static Joystick turtleBack = new Joystick(3);
  Button zeroTurret = new JoystickButton(turtleBack, 7);
  
  public static DoubleSupplier tail = () -> (turtleBack.getY()*0.75);//buttonBoardB.getY();//
  public static DoubleSupplier intake = () -> (turtleFront.getY());//buttonBoardB.getX(); //

  public static Joystick shoulderControl = new Joystick(5);
  public static DoubleSupplier shoulder = () -> (shoulderControl.getY());

  public static Joystick wristControl = new Joystick(5);
  public static DoubleSupplier wrist = () -> (wristControl.getY());

  public OI() {

    ////////////
    // STATES //
    ////////////

    ballFeedIn.whileHeld(ballFeed.setStateCommand(BallFeeder.State.FEED,BallFeeder.State.STOP, false));
    ballFeedOut.whileHeld(ballFeed.setStateCommand(BallFeeder.State.UNFEED,BallFeeder.State.STOP, false));

    hatchGrab.whileHeld(HAL.hatch.setStateCommand(HatchPatchSubsystem.State.OUT));
    hatchRelease.whileHeld(HAL.hatch.setStateCommand(HatchPatchSubsystem.State.IN));

    //switchPipeline.whileHeld(new SwitchPipeline(7));
    //trackPipeline.whenPressed(new SwitchPipeline(7));
    
    //drivePipeline.whenPressed(new SwitchPipeline(8));
    //climbStop.whenPressed(HAL.frontClimber.set);

    //////////////
    // COMMANDS //
    //////////////
    zeroTurret.whileHeld(new Zeros("turret"));
    
    travelPosition.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.TRAVEL,Shoulder.ShoulderPreset.TRAVEL,Elevator.ElevatorPreset.TRAVEL));    

    // GROUND
    ballFeedGround.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.GROUND_BALL,Shoulder.ShoulderPreset.GROUND_BALL,Elevator.ElevatorPreset.GROUND_BALL));
    
    // FEEDER STATION
    //ballFeederStation.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.FEEDER_STATION_BALL, Shoulder.ShoulderPreset.FEEDER_STATION_BALL,Elevator.ElevatorPreset.FEEDER_STATION_BALL));
    hatchFeederStation.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.FEEDER_STATION_HATCH, Shoulder.ShoulderPreset.FEEDER_STATION_HATCH, Elevator.ElevatorPreset.FEEDER_STATION_HATCH));

    /////////////////////////////
    // ROCKET & CARGO SHIP OLD //
    /////////////////////////////
    // lowBallRocket.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_LOW_BALL,Shoulder.ShoulderPreset.ROCKET_LOW_BALL,Elevator.ElevatorPreset.ROCKET_LOW_BALL));
    // midBallRocket.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_MID_BALL,Shoulder.ShoulderPreset.ROCKET_MID_BALL,Elevator.ElevatorPreset.ROCKET_MID_BALL));
    // highBallRocket.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_HIGH_BALL,Shoulder.ShoulderPreset.ROCKET_HIGH_BALL,Elevator.ElevatorPreset.ROCKET_HIGH_BALL));

    // lowHatchRocket.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_LOW_HATCH,Shoulder.ShoulderPreset.ROCKET_LOW_HATCH,Elevator.ElevatorPreset.ROCKET_LOW_HATCH));
    // midHatchRocket.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_MID_HATCH,Shoulder.ShoulderPreset.ROCKET_MID_HATCH,Elevator.ElevatorPreset.ROCKET_MID_HATCH));
    // highHatchRocket.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_HIGH_HATCH,Shoulder.ShoulderPreset.ROCKET_HIGH_HATCH,Elevator.ElevatorPreset.ROCKET_HIGH_HATCH));
    
    // cargoshipHatch.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_LOW_HATCH,Shoulder.ShoulderPreset.ROCKET_LOW_HATCH,Elevator.ElevatorPreset.ROCKET_LOW_HATCH));
    // cargoshipBall.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.CARGOSHIP_BALL, Shoulder.ShoulderPreset.CARGOSHIP_BALL, Elevator.ElevatorPreset.CARGOSHIP_BALL));

    ///////////////////////////////////
    // ROCKET & CARGO SHIP STAGGERED //
    ///////////////////////////////////
    lowBallRocketStag.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_LOW_BALL,Shoulder.ShoulderPreset.ROCKET_LOW_BALL,Elevator.ElevatorPreset.ROCKET_LOW_BALL));
    midBallRocketStag.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_MID_BALL,Shoulder.ShoulderPreset.ROCKET_MID_BALL,Elevator.ElevatorPreset.ROCKET_MID_BALL));
    highBallRocketStag.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_HIGH_BALL,Shoulder.ShoulderPreset.ROCKET_HIGH_BALL,Elevator.ElevatorPreset.ROCKET_HIGH_BALL));

    lowHatchRocketStag.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_LOW_HATCH,Shoulder.ShoulderPreset.ROCKET_LOW_HATCH,Elevator.ElevatorPreset.ROCKET_LOW_HATCH));
    midHatchRocketStag.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_MID_HATCH,Shoulder.ShoulderPreset.ROCKET_MID_HATCH,Elevator.ElevatorPreset.ROCKET_MID_HATCH));
    highHatchRocketStag.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_HIGH_HATCH,Shoulder.ShoulderPreset.ROCKET_HIGH_HATCH,Elevator.ElevatorPreset.ROCKET_HIGH_HATCH));
    
    cargoshipHatchStag.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.FEEDER_STATION_HATCH, Shoulder.ShoulderPreset.FEEDER_STATION_HATCH, Elevator.ElevatorPreset.FEEDER_STATION_HATCH));
    cargoshipBallStag.whenPressed(new Staggered_E_S_W(Wrist.WristPreset.CARGOSHIP_BALL, Shoulder.ShoulderPreset.CARGOSHIP_BALL, Elevator.ElevatorPreset.CARGOSHIP_BALL));    

    // CLIMBER
    frontClimbHome.whenPressed(new SetFrontClimber(FrontClimber.FrontClimberPreset.HOME));
    //frontClimbGround.whenPressed(new Climb(FrontClimber.FrontClimberPreset.AUTO_CLIMB,BackClimber.BackClimberPreset.AUTO_CLIMB,2));
    frontClimbPassLvl_3.whenPressed(new SetFrontClimber(FrontClimber.FrontClimberPreset.PASS3));
    frontClimbPassLvl_2.whenPressed(new SetFrontClimber(FrontClimber.FrontClimberPreset.PASS2));

    // TURRET
    turretFront.whileHeld(new SetTurret(Turret.TurretPreset.FRONT));
    turretRight.whileHeld(new SetTurret(Turret.TurretPreset.RIGHT));
    turretLeft.whileHeld(new SetTurret(Turret.TurretPreset.LEFT));
    turretBack.whileHeld(new SetTurret(Turret.TurretPreset.BACK));
    //turretRocketRightBack.whileHeld(new SetTurret(Turret.TurretPreset.RIGHT_ROCKET_BACK));

    anyTurret.whenReleased(new HoldTurret(OI.turret));
    
    // FRONT CLIMBER
    turretZero.whenPressed(new Zeros("turret"));
    //hatchToElevatorTransfer.whenPressed(new
    // SetElevatorShoulderWrist(Wrist.WristPreset.HATCH_GROUND_2_ELEVATOR_TRANSFER,Shoulder.ShoulderPreset.HATCH_GROUND_2_ELEVATOR_TRANSFER,Elevator.ElevatorPreset.HATCH_GROUND_2_ELEVATOR_TRANSFER));
    //---------------------
    //testing
    //--------------------
    climbAutoLvl_3.whenPressed(new Climb(FrontClimber.FrontClimberPreset.AUTO_CLIMB,BackClimber.BackClimberPreset.AUTO_CLIMB,3));
    climbAutoLvl_2.whenPressed(new Climb(FrontClimber.FrontClimberPreset.AUTO_CLIMB,BackClimber.BackClimberPreset.AUTO_CLIMB,2));
    climbStop.whenPressed(new StopClimb());

    // ShoulderNudges
    //shoulderUp.whenPressed(new ShoulderNudge(true));
    //shoulderDown.whenPressed(new ShoulderNudge(false));

    turretLime.whileHeld(new TurretAlignLime(1));
    highLime.whileHeld(new TurretAlignLime(2));
    turretLime.whileHeld(new SwitchPipeline(6));
    turretLime.whenReleased(new SwitchPipeline(8));
    highLime.whileHeld(new SwitchPipeline(6));
    highLime.whenReleased(new SwitchPipeline(8));

    // turretLime3.whileHeld(new TurretAlignLime());
    // turretLime3.whileHeld(new SwitchPipeline(6));
    // turretLime3.whenReleased(new SwitchPipeline(8));

    switchVision.whenPressed(new SwitchPipeline(8));

    // visionMidHigh.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.TRAVEL, Shoulder.ShoulderPreset.TRAVEL, Elevator.ElevatorPreset.VISION_TRAVEL));
    // visionMidHigh.whileHeld(new VisionAgainstRocket());
    
    visionTurret.whenPressed(new SetElevatorShoulderWrist(Wrist.WristPreset.TRAVEL, Shoulder.ShoulderPreset.TRAVEL, Elevator.ElevatorPreset.VISION_TRAVEL));
    visionTurret.whileHeld(new TurretAlignLime(1));
  }
}