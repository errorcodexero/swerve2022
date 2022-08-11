package org.xero1425.base.subsystems.intake2motor;

import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;

public class CollectOnAction extends MotorEncoderGotoAction {
    private Intake2MotorSubsystem sub_ ;
    private double collect_power_ ;

    public CollectOnAction(Intake2MotorSubsystem sub)  throws Exception {
        super(sub, "collect:onpos", true) ;

        sub_ = sub ;
        collect_power_ = sub.getSettingsValue("collector:onpower").getDouble() ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.setCollectorPower(collect_power_);
    }
    
    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public void cancel() {
        super.cancel();

        try {
            sub_.setCollectorPower(0.0);
        } 
        catch(Exception ex) {
        }
    }
}
