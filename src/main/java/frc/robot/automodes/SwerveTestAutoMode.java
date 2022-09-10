package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.subsystems.intake2motor.IntakePositionPowerAction;
import org.xero1425.base.subsystems.intake2motor.Intake2MotorSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.common.SwervePowerAngleAction;
import org.xero1425.base.subsystems.swerve.common.SwerveSpeedAngleAction;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
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

        Swerve2022RobotSubsystem robotsys = (Swerve2022RobotSubsystem) ctrl.getRobot().getRobotSubsystem();
        SwerveBaseSubsystem swerve = (SwerveBaseSubsystem) robotsys.getDB();
        GPMSubsystem gpm = robotsys.getGPM() ;
        MotorEncoderSubsystem wheels1 = gpm.getShooter().getWheel1Subsystem() ;
        MotorEncoderSubsystem wheels2 = gpm.getShooter().getWheel2Subsystem() ;
        MotorEncoderSubsystem hood = gpm.getShooter().getHoodSubsystem() ;
        Intake2MotorSubsystem intake = gpm.getIntake() ;
        MotorSubsystem agitator = gpm.getAgitator() ;
        ConveyorSubsystem conveyor = gpm.getConveyor(); 
        TurretSubsystem turret = robotsys.getTurret() ;

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
                addSubActionPair(swerve, new SwerveHolonomicPathFollower(swerve, getString("name"), getDouble("endangle")), true);
                break ;

            case 5:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                angles[0] = 0.0 ;
                angles[1] = 0.0 ;
                angles[2] = 0.0 ;
                angles[3] = 0.0 ;
                powers[0] = 0.0 ;
                powers[1] = 0.0 ;
                powers[2] = 0.0 ;
                powers[3] = 0.1 ;
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
                addSubActionPair(intake, new MotorEncoderGotoAction(intake, 6000, false), true);
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

            //
            // Shooter test modes
            //
            case 40:
                //
                addSubActionPair(conveyor, new MotorPowerAction(conveyor, 0.8), true) ;
                addSubActionPair(wheels1, new MotorEncoderPowerAction(wheels1, getDouble("power")), false) ;
                addSubActionPair(wheels2, new MotorEncoderPowerAction(wheels2, getDouble("power")), false) ;
                addAction(new DelayAction(ctrl.getRobot(), getDouble("duration"))) ;
                break ;

            case 41:
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 0.1), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 1.0)) ;
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 0.2), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 1.0)) ;
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 0.3), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 1.0)) ;
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 0.4), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 1.0)) ;
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 0.5), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 1.0)) ;
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 0.6), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 1.0)) ;
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 0.7), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 1.0)) ;
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 0.8), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 1.0)) ;
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 0.9), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 1.0)) ;
                // addSubActionPair(wheels, new MotorEncoderPowerAction(wheels, 1.0), false) ;
                // addAction(new DelayAction(ctrl.getRobot(), 111.0)) ;
                break ;
            
            //
            // Hood test modes
            //
            case 50:
                addSubActionPair(hood, new MotorEncoderPowerAction(hood, getDouble("power"), getDouble("duration")), true) ;
                break ;

            //
            // Turret test modes
            //
            case 60:
                addSubActionPair(turret, new MotorEncoderPowerAction(turret, getDouble("power"), getDouble("duration")), true) ;
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
