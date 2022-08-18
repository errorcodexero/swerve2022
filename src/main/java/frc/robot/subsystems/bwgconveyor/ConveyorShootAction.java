package frc.robot.subsystems.bwgconveyor;

import org.xero1425.base.actions.Action;

public class ConveyorShootAction extends Action {

    private ConveyorSubsystem sub_ ;
    
    public ConveyorShootAction(ConveyorSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;        
        sub_.shoot() ;
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
        super.cancel() ;
        sub_.stop();
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorCollectAction" ;
    }
}
