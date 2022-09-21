package frc.robot.subsystems.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorTestShootAction extends Action {
    private ConveyorSubsystem sub_ ;
    private double duration_ ;

    public ConveyorTestShootAction(ConveyorSubsystem sub, double duration) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        duration_ = Double.NaN ;
    }

    @Override
    public void start() throws Exception{
        super.start();

        sub_.shoot(duration_) ;
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
