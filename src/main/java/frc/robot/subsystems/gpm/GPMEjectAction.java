package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.intake2motor.IntakePositionPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;

public class GPMEjectAction extends Action{

    private GPMSubsystem subsystem_;
    private MotorPowerAction agitator_;
    private IntakePositionPowerAction intake_;
    public GPMEjectAction(GPMSubsystem subsystem) throws Exception {
        super(subsystem.getRobot().getMessageLogger());
        subsystem = subsystem_;

        agitator_ = new MotorPowerAction(subsystem_.getAgitator(), subsystem_.getAgitator().getSettingsValue("ejectpower").getDouble());
        intake_ = new IntakePositionPowerAction(subsystem_.getIntake(), "collect:onpos", "collector:ejectpower", false, false);

    }

    @Override
    public void start() throws Exception {
        super.start();
        subsystem_.getAgitator().setAction(agitator_, true);
        subsystem_.getIntake().setAction(intake_, true);
    }

    @Override
    public void cancel() {
        super.cancel();
    }


    @Override
    public String toString(int indent) {
        // TODO Auto-generated method stub
        return null;
    }


    
}
