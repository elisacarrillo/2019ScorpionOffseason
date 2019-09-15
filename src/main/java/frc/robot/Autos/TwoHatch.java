/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Autos;

import frc.robot.ninjaLib.PathLoader;
import frc.robot.commands.FollowTrajectory;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.HAL;
import frc.robot.subsystems.BallFeeder;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class TwoHatch extends CommandGroup {
  Waypoint[] StarttoSetupWP = new Waypoint[] {
    new Waypoint(5, 10, 0),
    new Waypoint(18, 7, Pathfinder.d2r(30)),
    new Waypoint(23, 3.5, 0)
    };

  Waypoint[] SetuptoRocketWP = new Waypoint[] {
    new Waypoint(23, 3.5, 0),
    new Waypoint(18, 7, Pathfinder.d2r(-20))
    };

  Waypoint[] RockettoSetupWP = new Waypoint[] {
    new Waypoint(18, 7, Pathfinder.d2r(160)),//Was -20 but added 180 since changed the front of robot
    new Waypoint(23, 3.5, 0),
    };
   
  Waypoint[] SetuptoFeedWP = new Waypoint[] {
    new Waypoint(23, 3.5, 0),
    new Waypoint(9.5, 4.5, 0),
    new Waypoint(0, 2, 0)
    };
  Waypoint[] FeedtoSetupWP = new Waypoint[] {
    new Waypoint(0, 2, 0),
    new Waypoint(9.5, 4.5, 0),
    new Waypoint(23, 3.5, 0)
    }; 
 // Waypoint[] SetuptoRocket2 = new Waypoint[] {
    // new Waypoint(23, 3.5, 0),
    // new Waypoint(9.5, 4.5, 0),
    // new Waypoint(0, 2, 0)
    // };  
        
Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
    0.02,   //delta time
    1.5,    //max velocity in ft/s for the motion profile
    1.5,    //max acceleration in ft/s/s for the motion profile
    500); //max jerk in ft/s/s/s for the motion profile


Trajectory StarttoSetup = Pathfinder.generate(StarttoSetupWP, config);
Trajectory SetuptoRocket = Pathfinder.generate(SetuptoRocketWP, config);
Trajectory RockettoSetup = Pathfinder.generate(RockettoSetupWP, config);
Trajectory SetuptoFeed = Pathfinder.generate(SetuptoFeedWP, config);
Trajectory FeedtoSetup = Pathfinder.generate(FeedtoSetupWP, config);

public TwoHatch(boolean backwards) {
  addSequential(new FollowTrajectory(StarttoSetup, true));
 // addParallel(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_LOW_HATCH, Shoulder.ShoulderPreset.ROCKET_LOW_HATCH, Elevator.ElevatorPreset.ROCKET_LOW_HATCH));
  addSequential(new FollowTrajectory(SetuptoRocket, false));
 // addSequential(new SetFeeder(BallFeeder.State.UNFEED, BallFeeder.State.STOP, 0.5));
  addSequential(new FollowTrajectory(RockettoSetup, true));
  addSequential(new FollowTrajectory(SetuptoFeed, false));
 // addSequential(new SetFeeder(BallFeeder.State.UNFEED, BallFeeder.State.STOP, 1.0));
  addSequential(new FollowTrajectory(FeedtoSetup, true));
  addSequential(new FollowTrajectory(SetuptoRocket, false));
 // addSequential(new SetFeeder(BallFeeder.State.UNFEED, BallFeeder.State.STOP, 0.5));
}
}

