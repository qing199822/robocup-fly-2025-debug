#include "tracking/broadcast_progress.h"

#include <cmath>
#include <stdexcept>

BroadcastProgress::BroadcastProgress(double confirmation_duration,
                                     double heartbeat_timeout,
                                     double session_timeout)
    : confirmation_duration_(confirmation_duration),
      heartbeat_timeout_(heartbeat_timeout),
      session_timeout_(session_timeout)
{
    if (!std::isfinite(confirmation_duration_) ||
        !std::isfinite(heartbeat_timeout_) ||
        !std::isfinite(session_timeout_) ||
        confirmation_duration_ <= 0.0 || heartbeat_timeout_ <= 0.0 ||
        session_timeout_ <= 0.0 ||
        confirmation_duration_ >= session_timeout_ ||
        heartbeat_timeout_ >= confirmation_duration_) {
        throw std::invalid_argument("invalid broadcast progress timing configuration");
    }
}

void BroadcastProgress::start(double now)
{
    if (!std::isfinite(now) || now <= 0.0) {
        throw std::invalid_argument("broadcast session start must be finite and positive");
    }

    reset();
    session_start_ = now;
    active_ = true;
}

bool BroadcastProgress::recordHeartbeat(double stamp, double now)
{
    if (!active_ || !std::isfinite(stamp) || !std::isfinite(now) ||
        stamp <= 0.0 || stamp > now ||
        (last_heartbeat_ > 0.0 && stamp < last_heartbeat_)) {
        return false;
    }

    if (last_heartbeat_ <= 0.0 ||
        stamp - last_heartbeat_ > heartbeat_timeout_) {
        streak_start_ = stamp;
    }
    last_heartbeat_ = stamp;

    if (stamp - streak_start_ >= confirmation_duration_) {
        broadcast_confirmed_ = true;
    }
    return true;
}

bool BroadcastProgress::active() const
{
    return active_;
}

bool BroadcastProgress::broadcastConfirmed() const
{
    return broadcast_confirmed_;
}

bool BroadcastProgress::sessionTimedOut(double now) const
{
    return active_ && std::isfinite(now) && now >= session_start_ &&
           now - session_start_ >= session_timeout_;
}

void BroadcastProgress::reset()
{
    session_start_ = 0.0;
    streak_start_ = 0.0;
    last_heartbeat_ = 0.0;
    active_ = false;
    broadcast_confirmed_ = false;
}
