package frc.robot.subsystems.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorTestShootAction extends Action {
    private ConveyorSubsystem sub_ ;

    public ConveyorTestShootAction(ConveyorSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
    }

    @Override
    public void start() throws Exception{
        super.start();

        sub_.testShoot() ;
    }

    @Override
    public void run() throws Exception{
        super.run() ;

        if (sub_.isIdle()) {
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorTestShootAction" ;
    }

    @Override
    public void cancel() {
        sub_.stop() ;
    }
}
