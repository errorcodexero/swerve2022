package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.gpm.GPMEjectAction;
import frc.robot.subsystems.gpm.GPMStartCollectAction;
import frc.robot.subsystems.gpm.GPMStopCollectAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.turret.TurretFollowTargetAction;

import org.xero1425.base.actions.Action;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OIPanelButton;

public class Swerve2022OIDevice extends OIPanel {
    private int automode_gadget_ ;
    private int collect_v_shoot_gadget_ ;
    private int shoot_manual_mode_gadget_ ;
    private int start_collect_gadget_ ;
    private int eject_gadget_ ;
    private int climb_lock_gadget_ ;

    private OILed ball1_output_ ;
    private OILed ball2_output_ ;

    private OILed limelight_ready_led_ ;
    private OILed shooter_ready_led_ ;
    private OILed turret_ready_led_ ;
    private OILed distance_ok_led_ ;

    private Action follow_action_ ;
    private Action gpm_eject_action_ ;
    private Action start_collect_action_ ;
    private Action stop_collect_action_ ;

    public Swerve2022OIDevice(OISubsystem parent, String name, int index) throws BadParameterTypeException, MissingParameterException {
        super(parent, name, index) ;

        initializeGadgets() ;

        ball1_output_ = createLED(parent.getSettingsValue("panel:outputs:ball1").getInteger()) ;
        ball2_output_ = createLED(parent.getSettingsValue("panel:outputs:ball2").getInteger()) ;

        limelight_ready_led_ = createLED(parent.getSettingsValue("panel:outputs:shooting:limelight").getInteger()) ;
        shooter_ready_led_ = createLED(parent.getSettingsValue("panel:outputs:shooting:shooter").getInteger()) ;        
        turret_ready_led_ = createLED(parent.getSettingsValue("panel:outputs:shooting:turret").getInteger()) ;
        distance_ok_led_ = createLED(parent.getSettingsValue("panel:outputs:shooting:distance").getInteger()) ;  
    }

    @Override
    public int getAutoModeSelector() {
        int value = getValue(automode_gadget_) ;
        return value ;
    }

    @Override
    public void createStaticActions() throws Exception {
        Swerve2022RobotSubsystem robot = (Swerve2022RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = robot.getGPM() ;

        gpm_eject_action_ = new GPMEjectAction(gpm) ;
        start_collect_action_ = new GPMStartCollectAction(gpm) ;
        stop_collect_action_ = new GPMStopCollectAction(gpm) ;
        // follow_action_ = new TurretFollowTargetAction(robot.getTurret(), robot.getTracker()) ;
    }


    @Override
    public void generateActions() throws InvalidActionRequest {

        Swerve2022RobotSubsystem robot = (Swerve2022RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = robot.getGPM() ;

        // First set the LEDs
        setLEDs() ;

        if (getValue(eject_gadget_) == 1) {
            if (gpm.getAction() != gpm_eject_action_)
                gpm.setAction(gpm_eject_action_);
        }
        else if (getValue(climb_lock_gadget_) == 1) {
            generateCargoActions() ;
        }
        else if (robot.getClimber() != null) {
            generateClimbActions() ;
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
            // TODO: Shooting
        }
    }

    private void generateClimbActions() {

    }

    private void setLEDs()
    {
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
        int num = getSubsystem().getSettingsValue("panel:gadgets:automode").getInteger();
        Double[] map = { -0.3, 0.05, 0.20, 0.30, 0.50, 0.60, 0.80, 0.9, 1.0} ;
        
        automode_gadget_ = mapAxisScale(num, map);

        num = getSubsystem().getSettingsValue("panel:gadgets:shoot_collect_mode").getInteger();
        collect_v_shoot_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:shoot_manual_mode").getInteger() ;
        shoot_manual_mode_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:collect_onoff").getInteger();
        start_collect_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:eject").getInteger();
        eject_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_lock").getInteger();
        climb_lock_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);
    }
}