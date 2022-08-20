package frc.robot.subsystems.bwgconveyor;

import org.xero1425.base.actions.Action;

public class ConveyorEjectAction extends Action {
    ConveyorSubsystem sub_ ;

    public ConveyorEjectAction(ConveyorSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        sub_.eject();
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (sub_.isIdle()) {
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel();
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorEjectAction" ;
    }
}
