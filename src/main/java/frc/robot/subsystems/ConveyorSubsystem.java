package frc.robot.subsystems;

import org.xero1425.base.motors.MotorController;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DigitalInput;

public class ConveyorSubsystem  extends MotorSubsystem {

   /* private static final int intake_sensor_idx = 0 ; 
    private static final int shooter_sensor_idx = 1 ;
    private static final int middle_sensor_idx = 2 ; */

    private DigitalInput intake_sensor_ ; 
    private DigitalInput shooter_sensor_ ;
    private DigitalInput middle_sensor_ ;

    private boolean intake_value_ = false;
    private boolean middle_value_ = false;
    private boolean shooter_value_ = false;

    private double collect_power_ ;
    private double off_power_ ;
    private double eject_power_ ;
    
    private MotorController conveyor_motor_ ;

    public ConveyorSubsystem(Subsystem parent) throws BadParameterTypeException, MissingParameterException {
        super(parent, "conveyor") ;
        collect_power_ = getSettingsValue("power:collect").getDouble() ;
        off_power_ = getSettingsValue("power:off").getDouble();
        eject_power_ = getSettingsValue("power:eject").getDouble(); 
    }

    public void ComputeMyState() {
        intake_value_ = !intake_sensor_.get(); 
        shooter_value_ = !shooter_sensor_.get();
        middle_value_ = !middle_sensor_.get();
    }

    private State state_ ;

    private enum State {
        Idle,
        WaitForIntake,
        WaitForMiddle,
        WaitForShooter,
        WaitForIntake2
    }
        public void run() {
        switch(state_) {

            case Idle:
    
            break ;
    
            case WaitForIntake:
    
            WaitForIntakeProc();

            break ;

            case WaitForMiddle:

            WaitForMiddleProc();

            break ;

            case WaitForShooter:

            WaitForShooterProc();

            break ;

            case WaitForIntake2:

            WaitForIntake2Proc();

            break;
    
        }
    }
    
        private void WaitForIntakeProc(){
    
            if (intake_value_ == true){
                setPower(collect_power_);
                state_ = State.WaitForMiddle;
            }
        }

        private void WaitForMiddleProc(){

            if (intake_value_ == true && middle_value_ == true){
                setPower(collect_power_);
                state_ = State.WaitForShooter ;
            }
                else if(middle_value_ == true){
                    setPower(off_power_) ;
                    state_ = State.WaitForIntake2 ;
                }
        }

        private void WaitForShooterProc(){
            if (shooter_value_ == true){
                setPower(off_power_) ;
                state_ = State.WaitForIntake ;
            }
        }

        private void WaitForIntake2Proc(){
            if(intake_value_ == true && middle_value_ == true){
                state_ = State.WaitForMiddle ;
            }
            else if(middle_value_ == true){
                state_ = State.WaitForIntake ;
            }
        }
 
}