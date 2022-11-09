package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.UpdatableTestAction;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.subsystems.intake2motor.IntakePositionPowerAction;
import org.xero1425.base.subsystems.intake2motor.IntakePowerPowerAction;
import org.xero1425.base.subsystems.intake2motor.Intake2MotorSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderTrackPositionAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderTrackVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.common.SwervePowerAngleAction;
import org.xero1425.base.subsystems.swerve.common.SwerveSpeedAngleAction;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSetBall;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.gpm.GPMEjectAction;
import frc.robot.subsystems.gpm.GPMStartCollectAction;
import frc.robot.subsystems.gpm.GPMStopCollectAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.gpm.GPMTestShooterAction;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SwerveTestAutoMode extends TestAutoMode {

    public SwerveTestAutoMode(SwerveDriveRobotAutoController ctrl) throws Exception {
        super(ctrl, "Swerver-Test-Mode");

        double[] angles = new double[4];
        double[] powers = new double[4];

        double [] test53_times = { 2.0, 2.0, 2.0, 2.0, 2.0} ;
        double [] test53_values = { 100, 600, 200, 700, 400} ;

        Swerve2022RobotSubsystem robotsys = (Swerve2022RobotSubsystem) ctrl.getRobot().getRobotSubsystem();
        SwerveBaseSubsystem swerve = (SwerveBaseSubsystem) robotsys.getDB();
        GPMSubsystem gpm = robotsys.getGPM() ;
        MotorEncoderSubsystem wheels = gpm.getShooter().getWheelSubsystem() ;
        MotorEncoderSubsystem hood = gpm.getShooter().getHoodSubsystem() ;
        Intake2MotorSubsystem intake = gpm.getIntake() ;
        MotorSubsystem agitator = gpm.getAgitator() ;
        ConveyorSubsystem conveyor = gpm.getConveyor(); 
        TurretSubsystem turret = robotsys.getTurret() ;
        ClimberSubsystem climber = robotsys.getClimber() ;

        switch (getTestNumber()) {
            case 0:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds)
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power")), true) ;
                break;

            case 1:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 2:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds).  Since speed is given, the PID controller will try to
                // maintain the target speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed")), true) ;
                break;

            case 3:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run  until the duration has expired.
                // Since speed is given, the PID controller will try to maintain the target speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed"), getDouble("duration")), true) ;
                break ;                

            case 4:
                // Run the path follower against the path given
                addSubActionPair(swerve, new SwerveHolonomicPathFollower(swerve, getString("name"), true), true);
                break ;

            case 5:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                angles[0] = getDouble("angle");
                angles[1] = getDouble("angle");
                angles[2] = getDouble("angle");
                angles[3] = getDouble("angle");
                powers[0] = getDouble("power");
                powers[1] = getDouble("power");
                powers[2] = getDouble("power");
                powers[3] = getDouble("power");
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, angles, powers, getDouble("duration")), true) ;
                break ;

            //
            // Intake test modes
            //
            case 10:
                addSubActionPair(intake, new IntakePositionPowerAction(intake,  "collect:onpos", "collector:onpower", false, false), false) ;
                addAction(new DelayAction(ctrl.getRobot(), getDouble("duration"))) ;
                addSubActionPair(intake, new IntakePositionPowerAction(intake, "collect:offpos", "collector:offpower", true, true), false) ;
                break ;

            case 11:
                addSubActionPair(intake, new MotorEncoderPowerAction(intake, getDouble("power"), getDouble("duration")), true);
                break ;

            case 12:
                addSubActionPair(intake, new MotorEncoderGotoAction(intake, getDouble("down"), false), true);
                addAction(new DelayAction(getAutoController().getRobot(), 3.00));
                addSubActionPair(intake, new MotorEncoderGotoAction(intake, getDouble("up"), false), true);
                break ;

            case 13:
                addSubActionPair(intake, new IntakePowerPowerAction(intake, getDouble("updown"), getDouble("spinner")), false) ;
                addAction(new DelayAction(getAutoController().getRobot(), getDouble("duration"))) ;
                addSubActionPair(intake, new IntakePowerPowerAction(intake, 0.0, 0.0), false) ;

                break ;
            
            //
            // Agitator test modes
            //
            case 20:
                addSubActionPair(agitator, new MotorPowerAction(agitator, getDouble("power"), getDouble("duration")), true) ;
                break ;

            //
            // Conveyor test modes
            //
            case 30:
                addSubActionPair(conveyor, new MotorPowerAction(conveyor, getDouble("power"), getDouble("duration")), true) ;
                break ;  
                
            case 31:
                addSubActionPair(conveyor, new ConveyorSetBall(conveyor), true);
                break ;

            //
            // Shooter test modes
            //
            case 40:
                //          
                addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, getDouble("power"), getDouble("duration")), false) ;
                break ;

            case 41:
                addSubActionPair(wheels, new MotorEncoderTrackVelocityAction(wheels, "wheels", getDouble("velocity")), true) ;
                break ;
            
            //
            // Hood test modes
            //
            case 50:
                addSubActionPair(hood, new MotorEncoderPowerAction(hood, getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 51:
                addSubActionPair(hood, new MotorEncoderTrackPositionAction(hood, "hoodpos", getDouble("position")), true);
                break ;

            case 52:
                addSubActionPair(hood, new MotorEncoderTrackPositionAction(hood, "hoodpos", 100), false);
                addAction(new DelayAction(getAutoController().getRobot(), 1.00));
                addSubActionPair(hood, new MotorEncoderTrackPositionAction(hood, "hoodpos", 600), false);
                addAction(new DelayAction(getAutoController().getRobot(), 1.00));
                addSubActionPair(hood, new MotorEncoderTrackPositionAction(hood, "hoodpos", 200), false);
                addAction(new DelayAction(getAutoController().getRobot(), 1.00));
                break ;

            case 53:
                addAction(new UpdatableTestAction(hood, new MotorEncoderTrackPositionAction(hood, "hoodpos", 0.0), test53_times, test53_values)) ;
                break ;

            //
            // Turret test modes
            //
            case 60:
                addSubActionPair(turret, new MotorEncoderPowerAction(turret, getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 61:
                addSubActionPair(turret, new MotorEncoderTrackPositionAction(turret, "follow", getDouble("angle")), true) ;
                break ;    

            case 62:
                addSubActionPair(turret, new MotorEncoderTrackPositionAction(turret, "follow", -15.0), false) ;
                addAction(new DelayAction(ctrl.getRobot(), getDouble("delay")));
                addSubActionPair(turret, new MotorEncoderTrackPositionAction(turret, "follow", 15.0), false) ;
                addAction(new DelayAction(ctrl.getRobot(), getDouble("delay")));
                addSubActionPair(turret, new MotorEncoderTrackPositionAction(turret, "follow", 0.0), false) ;

                break ;    

            //
            // Climber test mode
            //
            case 70:
                addSubActionPair(climber, new MotorEncoderPowerAction(climber, getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 71:
                addSubActionPair(climber, new MotorEncoderGotoAction(climber, getDouble("up"), false), true);
                addAction(new DelayAction(getAutoController().getRobot(), 1.00));
                addSubActionPair(climber, new MotorEncoderGotoAction(climber, getDouble("down"), false), true);
                break ;
                          
            //
            // GPM test modes
            //
            case 80:
                addSubActionPair(gpm, new GPMStartCollectAction(gpm), false);
                addAction(new DelayAction(ctrl.getRobot(), getDouble("delay")));
                addSubActionPair(gpm, new GPMStopCollectAction(gpm), false);                
                break ;

            case 81:
                addSubActionPair(gpm, new GPMEjectAction(gpm), false);
                break;     
                
            case 82:
                addSubActionPair(gpm, new GPMStartCollectAction(gpm), true); 
                addAction(new DelayAction(ctrl.getRobot(), getDouble("delay")));           
                addSubActionPair(gpm, new GPMEjectAction(gpm), true);
                break; 

            case 83:
                addSubActionPair(gpm, new GPMTestShooterAction(gpm), true) ;
                break ;
        }
    }
}
