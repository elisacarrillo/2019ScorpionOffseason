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



public class TurnTest extends CommandGroup {
  Waypoint[] points = new Waypoint[] {
    new Waypoint(0, 4, 0),
    new Waypoint(3, 4, 0),
    new Waypoint(4, 3, 90)
    };

Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
    0.02,   //delta time
    1.5,    //max velocity in ft/s for the motion profile
    1.5,    //max acceleration in ft/s/s for the motion profile
    500); //max jerk in ft/s/s/s for the motion profile


Trajectory trajectory = Pathfinder.generate(points, config);

public TurnTest(boolean backwards) {
  addSequential(new FollowTrajectory(trajectory, true));
  addSequential(new FollowTrajectory(trajectory, false));
}
}
