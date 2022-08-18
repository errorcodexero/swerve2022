package frc.robot.subsystems.bwgconveyor;

import org.xero1425.base.actions.Action;

public class ConveyorCollectAction extends Action {
    ConveyorSubsystem sub_ ;

    public ConveyorCollectAction(ConveyorSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        
        sub_.collect();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        
        if (sub_.isIdle())
            setDone() ;
    }

    @Override
    public void cancel() {
        sub_.stop() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorCollectAction" ;
    }
}
