package frc.robot.subsystems.conveyor;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.MessageLogger;


public class ConveyorSetBall extends Action {
    ConveyorSubsystem subsystem_;


    public ConveyorSetBall(ConveyorSubsystem subsystem) {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
    }


    @Override
    public void start() {
        subsystem_.setPreloadedBall();

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorSetBall";
    }

}