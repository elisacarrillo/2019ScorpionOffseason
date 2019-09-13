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



public class TwoHatch extends CommandGroup {
  Waypoint[] points = new Waypoint[] {
    new Waypoint(5, 10, 0),
    new Waypoint(18, 7, Pathfinder.d2r(30)),
    new Waypoint(23, 3.5, 0)
    };

  Waypoint[] points2 = new Waypoint[] {
    new Waypoint(23, 3.5, 0),
    new Waypoint(18, 7, Pathfinder.d2r(-20))
    };

  Waypoint[] points3 = new Waypoint[] {
    new Waypoint(23, 3.5, 0),
    new Waypoint(9.5, 4.5, 0),
    new Waypoint(0, 2, 0)
    };
        
Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
    0.02,   //delta time
    1.5,    //max velocity in ft/s for the motion profile
    1.5,    //max acceleration in ft/s/s for the motion profile
    500); //max jerk in ft/s/s/s for the motion profile


Trajectory trajectory = Pathfinder.generate(points, config);
Trajectory trajectory2 = Pathfinder.generate(points2, config);
Trajectory trajectory3 = Pathfinder.generate(points3, config);

public TwoHatch(boolean backwards) {
  addSequential(new FollowTrajectory(trajectory, true));
  addSequential(new FollowTrajectory(trajectory2, false));
 // addParallel(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_LOW_HATCH,Shoulder.ShoulderPreset.ROCKET_LOW_HATCH,Elevator.ElevatorPreset.ROCKET_LOW_HATCH));
  addSequential(new FollowTrajectory(trajectory2, true));
  addSequential(new FollowTrajectory(trajectory3, false));
 // addParallel(new SetElevatorShoulderWrist(Wrist.WristPreset.FEEDER_STATION_HATCH, Shoulder.ShoulderPreset.FEEDER_STATION_HATCH, Elevator.ElevatorPreset.FEEDER_STATION_HATCH));
  addSequential(new FollowTrajectory(trajectory3, true));
  addSequential(new FollowTrajectory(trajectory2, false));
}
}
