package org.xero1425.base.actions;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class UpdatableTestAction extends Action {

    private class TimeValue
    {
        public double time_ ;
        public double value_ ;

        public TimeValue(double t, double v) {
            time_ = t ;
            value_ = v ;
        }
    }

    private Subsystem sub_ ;
    private Action child_ ;
    private List<TimeValue> values_ ;
    private double start_time_ ;
    private int index_ ;

    public UpdatableTestAction(Subsystem sub, Action child) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        child_ = child ;
        values_ = new ArrayList<TimeValue>() ;
    }

    public UpdatableTestAction(Subsystem sub, Action child, double [] times, double [] values) throws Exception {
        this(sub, child) ;

        if (times.length != values.length) {
            throw new Exception("UpdatableTestAction expected times array and values array to be the same size") ;
        }

        for(int i = 0 ; i < times.length ; i++) {
            TimeValue tv = new TimeValue(times[i], values[i]) ;
            values_.add(tv) ;
        }
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (values_.size() == 0) {
            setDone() ;
        }
        else {
            MessageLogger logger = sub_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("Starting UpdatableTestAction for action '" + child_.toString(0) + "'") ;
            logger.add("value ", values_.get(0).value_) ;
            logger.endMessage(); 

            index_ = 0 ;
            start_time_ = sub_.getRobot().getTime() ;
            sub_.setAction(child_, true) ;
            child_.update(values_.get(0).value_) ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        double elapsed = sub_.getRobot().getTime() - start_time_ ;
        if (elapsed > values_.get(index_).time_) {
            index_++ ;
            if (index_ == values_.size()) {
                MessageLogger logger = sub_.getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Info) ;
                logger.add("Completed action '" + child_.toString(0) + "'") ;
                logger.endMessage(); 
                child_.cancel() ;
                setDone();
            }
            else {
                MessageLogger logger = sub_.getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Info) ;
                logger.add("Updating target for action '" + child_.toString(0) + "'") ;
                logger.add(", new target", values_.get(index_).value_) ;
                logger.endMessage(); 
                child_.update(values_.get(index_).value_) ;
                start_time_ = sub_.getRobot().getTime() ;
            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "UpdatableTestAction";
    }
}
