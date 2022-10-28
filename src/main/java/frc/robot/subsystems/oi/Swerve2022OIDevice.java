package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OILed.State;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.gpm.GPMEjectAction;
import frc.robot.subsystems.gpm.GPMFireAction;
import frc.robot.subsystems.gpm.GPMStartCollectAction;
import frc.robot.subsystems.gpm.GPMStopCollectAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.turret.TurretFollowTargetAction;

import org.xero1425.base.actions.Action;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.motors.MotorController.NeutralMode;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderTrackPositionAction;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OIPanelButton;

public class Swerve2022OIDevice extends OIPanel {
    private int collect_v_shoot_gadget_ ;
    private int start_collect_gadget_ ;
    private int eject_gadget_ ;
    private int climb_lock_gadget_ ;
    private int climb_deploy_gadget_;
    private int start_climb_gadget_;
    private int abort_climb_gadget_ ;

    private int stow_position_ ;
    private int deploy_position_ ;
    private int threshold_ ;
    private boolean climbing_ ;

    private OILed ball1_output_ ;
    private OILed ball2_output_ ;

    private OILed limelight_ready_led_ ;
    private OILed shooter_ready_led_ ;
    private OILed hood_ready_led_ ;
    private OILed turret_ready_led_ ;

    private Action follow_action_ ;
    private Action gpm_eject_action_ ;
    private Action start_collect_action_ ;
    private Action stop_collect_action_ ;
    private Action fire_action_ ;
    private Action climber_stow_action_;
    private Action climber_deploy_action_;
    private Action climber_climb_action_;
    private Action turret_zero_action_ ;

    public Swerve2022OIDevice(OISubsystem parent, String name, int index) throws BadParameterTypeException, MissingParameterException {
        super(parent, name, index) ;

        initializeGadgets() ;

        ball1_output_ = createLED(parent.getSettingsValue("panel:outputs:ball1").getInteger()) ;
        ball2_output_ = createLED(parent.getSettingsValue("panel:outputs:ball2").getInteger()) ;

        limelight_ready_led_ = createLED(parent.getSettingsValue("panel:outputs:shooting:limelight").getInteger()) ;
        shooter_ready_led_ = createLED(parent.getSettingsValue("panel:outputs:shooting:shooter").getInteger()) ;        
        turret_ready_led_ = createLED(parent.getSettingsValue("panel:outputs:shooting:turret").getInteger()) ;
        hood_ready_led_ = createLED(parent.getSettingsValue("panel:outputs:shooting:hood").getInteger()) ;

        ISettingsSupplier settings = parent.getRobot().getSettingsSupplier() ;
        stow_position_ = settings.get("subsystems:climber:stow-position").getInteger() ;
        deploy_position_ = settings.get("subsystems:climber:deploy-position").getInteger() ;
        threshold_ = settings.get("subsystems:climber:threshold").getInteger() ;
        
        climbing_ = false ;
    }

    @Override
    public void createStaticActions() throws Exception {
        Swerve2022RobotSubsystem robot = (Swerve2022RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = robot.getGPM() ;
        ClimberSubsystem climber = robot.getClimber();

        gpm_eject_action_ = new GPMEjectAction(gpm) ;
        start_collect_action_ = new GPMStartCollectAction(gpm) ;
        stop_collect_action_ = new GPMStopCollectAction(gpm) ;
        fire_action_ = new GPMFireAction(gpm, robot.getTracker(), robot.getDB(), robot.getTurret()) ;
        follow_action_ = new TurretFollowTargetAction(robot.getTurret(), robot.getTracker()) ;
        turret_zero_action_ = new MotorEncoderTrackPositionAction(robot.getTurret(), "follow", 0) ;

        if (climber != null) {
            climber_deploy_action_ = new MotorEncoderGotoAction(climber, "deploy-position", true);
            climber_stow_action_ = new MotorEncoderGotoAction(climber, "stow-position", true);
            climber_climb_action_ = new MotorEncoderHoldAction(climber, "climbpid", "climb-position") ;
        }
    }

    @Override
    public void generateActions() throws InvalidActionRequest {

        Swerve2022RobotSubsystem robot = (Swerve2022RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = robot.getGPM() ;

        // First set the LEDs
        setBallCountLEDs() ;
        setShootLEDs() ;

        if (getValue(eject_gadget_) == 1) {
            if (gpm.getAction() != gpm_eject_action_)
                gpm.setAction(gpm_eject_action_);
        }
        else if (gpm.getAction() == gpm_eject_action_) {
            //
            // We do nothing and wait for the eject action to complete
            //
        }
        else {
            if (getValue(climb_lock_gadget_) == 1) {
                generateCargoActions() ;
            }
            else if (robot.getClimber() != null) {
                generateClimbActions() ;
            }
        }
    }

    private void generateCargoActions() {
        Swerve2022RobotSubsystem robot = (Swerve2022RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = robot.getGPM() ;

        if (robot.getTurret() != null) {
            if (robot.getTurret().getAction() != follow_action_)
                robot.getTurret().setAction(follow_action_) ;
        }

        if (getValue(collect_v_shoot_gadget_) == 1) {
            //
            // We are in collect mode
            //
            if (isCollectButtonPressed()) {
                if (!gpm.getConveyor().isFull() && gpm.getAction() != start_collect_action_) {
                    gpm.setAction(start_collect_action_) ;
                }
            }
            else {
                if (gpm.getAction() != null && gpm.getAction() != stop_collect_action_)
                    gpm.setAction(stop_collect_action_) ;
            }
        }
        else {
            //
            // We are in shoot mode, if the GPM does not have the fire action, put it in place
            //
            if (gpm.getConveyor().getBallCount() > 0) {
                if (gpm.getAction() != fire_action_) {
                    gpm.setAction(fire_action_) ;
                }
            }
        }
    }

    private boolean isInPosition(int pos) {
        Swerve2022RobotSubsystem robot = (Swerve2022RobotSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        ClimberSubsystem climber = (ClimberSubsystem) robot.getClimber();
        return Math.abs(climber.getPosition() - pos) < threshold_ ;
    }

    private void generateClimbActions() {
        Swerve2022RobotSubsystem robot = (Swerve2022RobotSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        ClimberSubsystem climber = (ClimberSubsystem) robot.getClimber();

        if (robot.getTurret().getAction() != turret_zero_action_) {
            robot.getTurret().setAction(turret_zero_action_) ;
        }

        MessageLogger logger = robot.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
        logger.add("Climber OI:") ;
        logger.add("pos", climber.getPosition()) ;
        logger.add(", deployPosition", deploy_position_) ;
        logger.add(", stowPosition", stow_position_) ;
        logger.add(", button", getValue(climb_deploy_gadget_)) ;
        logger.add(", pos", isInPosition(deploy_position_)) ;
        logger.endMessage();

        if (getValue(climb_deploy_gadget_) == 1 && !isInPosition(deploy_position_) &&  !climbing_) {
            if (climber.getAction() != climber_deploy_action_) {
                climber.setAction(climber_deploy_action_);
            }
        }
        else if (!climbing_ && getValue(climb_deploy_gadget_) == 0 && !isInPosition(stow_position_)) {
            if (climber.getAction() != climber_stow_action_) {
                climber.setAction(climber_stow_action_);
            }
        }
        else if (getValue(start_climb_gadget_) == 1 && isInPosition(deploy_position_)) {            
            climbing_ = true ;

            try {
                climber.getMotorController().setNeutralMode(NeutralMode.Brake);
            }
            catch(Exception ex) {

            }

            if (climber.getAction() != climber_climb_action_) {
                climber.setAction(climber_climb_action_) ;
            }
        }
        else if (getValue(abort_climb_gadget_) == 1 && climbing_) {
            try {
                climber.getMotorController().setNeutralMode(NeutralMode.Coast);
            }
            catch(Exception ex) {
            }
            climbing_ = false ;
            if (climber.getAction() != climber_deploy_action_) {
                climber.setAction(climber_deploy_action_) ;
            }
        }
        logger.endMessage();
    }

    private void setShootLEDs() {
        Swerve2022RobotSubsystem robot = (Swerve2022RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = robot.getGPM() ;
        
        if (gpm.getAction() instanceof GPMFireAction) {
            GPMFireAction fire = (GPMFireAction)gpm.getAction() ;

            if (fire.shooting()) {
                turret_ready_led_.setState(State.BLINK_SLOW);
                shooter_ready_led_.setState(State.BLINK_SLOW);
                hood_ready_led_.setState(State.BLINK_SLOW);
                limelight_ready_led_.setState(State.BLINK_SLOW);
            }
            else {
                if (!fire.turretReady()) {
                    turret_ready_led_.setState(State.BLINK_FAST);
                }
                else {
                    turret_ready_led_.setState(State.ON);
                }

                if (!fire.shooterWheelsReady()) {
                    shooter_ready_led_.setState(State.BLINK_FAST);
                }
                else {
                    shooter_ready_led_.setState(State.ON);
                }

                if (!fire.shooterHoodReady()) {
                    hood_ready_led_.setState(State.BLINK_FAST);
                }
                else {
                    hood_ready_led_.setState(State.ON);
                }

                if (!fire.targetReady()) {
                    limelight_ready_led_.setState(State.BLINK_FAST);
                }
                else {
                    limelight_ready_led_.setState(State.ON);
                }   
            }
        }
        else {
            turret_ready_led_.setState(State.OFF);
            shooter_ready_led_.setState(State.OFF);
            limelight_ready_led_.setState(State.OFF) ;
            hood_ready_led_.setState(State.OFF) ;
        }
    }

    private void setBallCountLEDs() {
        Swerve2022RobotSubsystem robot = (Swerve2022RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
        ConveyorSubsystem sub = robot.getGPM().getConveyor() ;
        
        switch(sub.getBallCount())
        {
            case 0:
                ball1_output_.setState(State.OFF);
                ball2_output_.setState(State.OFF);
                break ;

            case 1:
                ball1_output_.setState(State.ON);
                ball2_output_.setState(State.OFF);
                break ;   
                
            case 2:
                ball1_output_.setState(State.ON);
                ball2_output_.setState(State.ON);
                break ;     
                
            default:
                ball1_output_.setState(State.BLINK_FAST);
                ball2_output_.setState(State.BLINK_FAST);
            break ;                 
        }
    }

    private boolean isCollectButtonPressed() {
        if (getValue(start_collect_gadget_) == 1)
            return true ;

        Gamepad g = getSubsystem().getGamePad() ;
        if (g != null && g.isRTriggerPressed())
            return true ;

        return false ;
    }

    private void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        int num ;
        
        num = getSubsystem().getSettingsValue("panel:gadgets:shoot_collect_mode").getInteger();
        collect_v_shoot_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:collect_onoff").getInteger();
        start_collect_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:eject").getInteger();
        eject_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:climb-lock").getInteger();
        climb_lock_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:deploy-climber").getInteger();
        climb_deploy_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:start-climb").getInteger();
        start_climb_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);    
        
        num = getSubsystem().getSettingsValue("panel:gadgets:abort-climb").getInteger();
        abort_climb_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);  
    }
}
