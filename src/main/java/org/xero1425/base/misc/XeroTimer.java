package org.xero1425.base.misc;

import org.xero1425.base.XeroRobot;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class XeroTimer {
    private static int LoggerID = -1 ;
    private static final String LoggerIDName = "XeroTimer" ;
    private XeroRobot robot_ ;
    private boolean running_ ;
    private double duration_ ;
    private double endtime_ ;
    private String name_ ;

    public XeroTimer(XeroRobot robot, String name, double duration) {
        robot_ = robot ;
        name_ = name ;
        duration_ = duration ;
        running_ = false ;
        endtime_ = 0.0 ;        

        if (LoggerID == -1) {
            LoggerID = robot_.getMessageLogger().registerSubsystem(LoggerIDName) ;
        }
    }

    public void start() {
        if (running_) {
            MessageLogger logger = robot_.getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("Timer ").add(name_).add(" was started while running").endMessage();
        }

        running_ = true ;
        endtime_ = robot_.getTime() + duration_ ;
    }

    public boolean isExpired() {
        boolean ret = false ;

        if (running_ && robot_.getTime() > endtime_) {
            running_ = false ;
            ret = true ;
        }

        return ret ;
    }
}