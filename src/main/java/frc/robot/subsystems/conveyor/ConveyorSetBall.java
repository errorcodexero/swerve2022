package frc.robot.subsystems.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorSetBall extends Action {
    ConveyorSubsystem subsystem_;

    public ConveyorSetBall(ConveyorSubsystem subsystem) {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        subsystem_.setPreloadedBall();
    }

    @Override
    public boolean isDone() {
        return subsystem_.isIdle() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorSetBall";
    }
}
