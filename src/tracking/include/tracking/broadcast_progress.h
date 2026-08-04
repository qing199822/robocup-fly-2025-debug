#ifndef TRACKING_BROADCAST_PROGRESS_H
#define TRACKING_BROADCAST_PROGRESS_H

class BroadcastProgress {
public:
    BroadcastProgress(double confirmation_duration,
                      double heartbeat_timeout,
                      double session_timeout);

    void start(double now);
    bool recordHeartbeat(double stamp, double now);
    bool active() const;
    bool broadcastConfirmed() const;
    bool sessionTimedOut(double now) const;
    void reset();

private:
    double confirmation_duration_;
    double heartbeat_timeout_;
    double session_timeout_;
    double session_start_ = 0.0;
    double streak_start_ = 0.0;
    double last_heartbeat_ = 0.0;
    bool active_ = false;
    bool broadcast_confirmed_ = false;
};

#endif  // TRACKING_BROADCAST_PROGRESS_H
