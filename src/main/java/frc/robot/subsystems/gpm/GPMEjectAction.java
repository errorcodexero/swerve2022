package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.intake2motor.IntakePositionPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;
import frc.robot.subsystems.conveyor.ConveyorEjectAction;

public class GPMEjectAction extends Action{
    private enum EjectState
    {
        IntakeGoingDown,
        Ejecting,
        IntakeGoingUp
    }

    private GPMSubsystem subsystem_;
    private MotorPowerAction agitator_;
    private IntakePositionPowerAction intake_down_;
    private IntakePositionPowerAction intake_up_;
    private ConveyorEjectAction conveyor_ ;
    private EjectState state_ ;

    public GPMEjectAction(GPMSubsystem subsystem) throws Exception {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;

        agitator_ = new MotorPowerAction(subsystem_.getAgitator(), subsystem_.getAgitator().getSettingsValue("ejectpower").getDouble());
        intake_down_ = new IntakePositionPowerAction(subsystem_.getIntake(), "collect:onpos", "collector:ejectpower", false, false);
        intake_up_ = new IntakePositionPowerAction(subsystem_.getIntake(), "collect:offpos", "collector:offpower", false, false);
        conveyor_ = new ConveyorEjectAction(subsystem_.getConveyor()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        state_ = EjectState.IntakeGoingDown ;
        subsystem_.getIntake().setAction(intake_down_, true);
    }

    @Override
    public void run() throws Exception {

        switch(state_) {
            case IntakeGoingDown:
                if (intake_down_.isDone()) {
                    //
                    // The intake is down, start the agitator and the conveyor
                    //
                    state_ = EjectState.Ejecting ;

                    subsystem_.getConveyor().setAction(conveyor_, true) ;
                    subsystem_.getAgitator().setAction(agitator_, true) ;
                }
                break ;

            case Ejecting:
                if (conveyor_.isDone()) {
                    state_ = EjectState.IntakeGoingUp ;
                    agitator_.cancel() ;
                    subsystem_.getIntake().setAction(intake_up_, true) ;
                }
                break ;

            case IntakeGoingUp:
                if (intake_up_.isDone()) {
                    setDone() ;
                }
                break ;
        }
    }

    @Override
    public void cancel() {
        super.cancel();
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMEjectAction";
    }
}
