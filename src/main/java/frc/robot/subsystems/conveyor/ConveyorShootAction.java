package frc.robot.subsystems.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorShootAction extends Action {

    private ConveyorSubsystem subsystem_;

    public ConveyorShootAction(ConveyorSubsystem subsystem) {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
    }

    @Override
    public void start() throws Exception {
        super.start();
        subsystem_.shoot();
    }

    @Override
    public void run() throws Exception {
        super.run();
        if (subsystem_.isIdle()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorShootAction";
    }
}
