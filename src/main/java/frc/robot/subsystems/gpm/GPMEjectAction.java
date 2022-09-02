package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.intake2motor.IntakePositionPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;
import frc.robot.subsystems.conveyor.ConveyorEjectAction;

public class GPMEjectAction extends Action{

    private GPMSubsystem subsystem_;
    private MotorPowerAction agitator_;
    private IntakePositionPowerAction intake_;
    private ConveyorEjectAction conveyor_ ;

    public GPMEjectAction(GPMSubsystem subsystem) throws Exception {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;

        agitator_ = new MotorPowerAction(subsystem_.getAgitator(), subsystem_.getAgitator().getSettingsValue("ejectpower").getDouble());
        intake_ = new IntakePositionPowerAction(subsystem_.getIntake(), "collect:onpos", "collector:ejectpower", false, false);
        conveyor_ = new ConveyorEjectAction(subsystem_.getConveyor()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        subsystem_.getAgitator().setAction(agitator_, true);
        subsystem_.getIntake().setAction(intake_, true);
        subsystem_.getConveyor().setAction(conveyor_, true) ;
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
