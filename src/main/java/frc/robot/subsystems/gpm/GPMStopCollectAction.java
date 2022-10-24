package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.intake2motor.IntakePositionPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;

public class GPMStopCollectAction extends Action {
    public GPMSubsystem subsystem_;
    public IntakePositionPowerAction intake_stop_action_;
    public MotorPowerAction agitator_off_action_;

    public GPMStopCollectAction(GPMSubsystem subsystem) throws Exception {
        super(subsystem.getRobot().getMessageLogger());

        this.subsystem_ = subsystem;
        intake_stop_action_ = new IntakePositionPowerAction(subsystem.getIntake(), "collect:offpos", "collector:offpower", true, true);
        agitator_off_action_ =  new MotorPowerAction(subsystem_.getAgitator(), subsystem_.getAgitator().getSettingsValue("offpower").getDouble());
    }

    @Override
    public void start() throws Exception {
        super.start();

        subsystem_.getIntake().setAction(intake_stop_action_,true);
        subsystem_.getAgitator().setAction(agitator_off_action_, true);
        subsystem_.getConveyor().stop() ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (subsystem_.getConveyor().isIdle() && intake_stop_action_.isDone()) {
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel();

        intake_stop_action_.cancel();
        agitator_off_action_.cancel();
    }
    
    @Override
    public String toString(int indent) {
        // README: Hollister, need to fill in these toString methods.  This is how we get good
        //         readable results in the log file
        return spaces(indent) + "GPMStopCollectAction" ;
    }    
}
