package frc.robot.subsystems.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorEjectAction extends Action {

    private ConveyorSubsystem subsystem_;

    public ConveyorEjectAction(ConveyorSubsystem subsystem) {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
    }

    @Override
    public void start() throws Exception {
        super.start();
        subsystem_.eject();
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
        // From Butch: you must return a string here
        return spaces(indent) + "ConveyorEjectActions";
    }
}
