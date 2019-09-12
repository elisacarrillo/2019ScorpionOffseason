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



public class TestSandstormExample extends CommandGroup {
  Waypoint[] points = new Waypoint[] {
    new Waypoint(0, 0, 0),
    new Waypoint(10, 0, 0)
    };

Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
    0.02, 	//delta time
    1.0,		//max velocity in ft/s for the motion profile
    0.5,		//max acceleration in ft/s/s for the motion profile
    5.0);	//max jerk in ft/s/s/s for the motion profile


Trajectory trajectory = Pathfinder.generate(points, config);

public TestSandstormExample(boolean backwards) {
  addSequential(new FollowTrajectory(trajectory, false));
}
}
