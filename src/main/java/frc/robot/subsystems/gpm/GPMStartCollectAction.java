package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.intake2motor.IntakePositionPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;

import frc.robot.subsystems.bwgconveyor.ConveyorCollectAction;

public class GPMStartCollectAction extends Action {
    private GPMSubsystem subsystem_;
    private IntakePositionPowerAction intake_on_action_;
    private MotorPowerAction agitator_on_action_;
    private ConveyorCollectAction conveyor_on_action_ ;

    public GPMStartCollectAction(GPMSubsystem subsystem) throws Exception {
        super(subsystem.getRobot().getMessageLogger());
        this.subsystem_ = subsystem;

        intake_on_action_ = new IntakePositionPowerAction(subsystem_.getIntake(), "collect:onpos", "collect:offpos", false, false);
        agitator_on_action_ = new MotorPowerAction(subsystem_.getAgitator(), subsystem_.getAgitator().getSettingsValue("forwardpower").getDouble()); // maybe put power in params file
        conveyor_on_action_ = new ConveyorCollectAction(subsystem_.getConveyor()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        subsystem_.getIntake().setAction(intake_on_action_);
        subsystem_.getAgitator().setAction(agitator_on_action_);
        subsystem_.getConveyor().setAction(conveyor_on_action_) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        // README: Hollister - note the GPM start collection is done, when the conveyor collect action is done
        if (conveyor_on_action_.isDone())
            setDone() ;
    }

    @Override
    public void cancel() {
        super.cancel();
        intake_on_action_.cancel();
        agitator_on_action_.cancel();
    }

    @Override
    public String toString(int indent) {
        // README: Hollister, need to fill in these toString methods.  This is how we get good
        //         readable results in the log file
        return spaces(indent) + "GPMStartCollectAction" ;
    }
}
