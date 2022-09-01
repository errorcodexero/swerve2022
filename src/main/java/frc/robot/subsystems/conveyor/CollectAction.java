package frc.robot.subsystems.conveyor;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.MessageLogger;

public class CollectAction extends Action{

    private ConveyorSubsystem subsystem_; 

    public CollectAction(ConveyorSubsystem subsystem) {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem; 
    }

    @Override
    public void start() throws Exception {
    super.start();
    subsystem_.collect();
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
        return null;
    }
    
}
