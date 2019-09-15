package frc.robot.commands;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.HAL;

import static frc.robot.HAL.ballFeed;

import frc.robot.subsystems.BallFeeder;
import frc.robot.subsystems.BallFeeder.State;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetFeeder extends Command {
	private BallFeeder.State startState;
	private BallFeeder.State endState;
    private boolean ends;
    double timeout;
	  
    public SetFeeder(BallFeeder.State startState, BallFeeder.State endState, double timeout) {
    	super(2);
    	this.startState = startState;
    	this.endState = endState;
    	this.ends = true;
    //    this(() -> startState, endState, timeout);
    }
    
    @Override
    protected boolean isFinished() {
        return ends && this.isTimedOut();
    }

    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
    @Override
    protected void interrupted() {
    	end();
    }
}
