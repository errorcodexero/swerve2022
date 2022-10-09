package frc.robot.subsystems.shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class ShooterSpinUpAction extends Action {
    private ShooterSubsystem sub_ ;
    private double spinup_power_ ;

    public ShooterSpinUpAction(ShooterSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        spinup_power_ = sub_.getSettingsValue("spinup-power").getDouble() ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getWheelSubsystem().setPower(spinup_power_) ;
        setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ShooterSpinUpAction" ;
    }
}
