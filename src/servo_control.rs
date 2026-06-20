#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ServoPhase {
    Idle,
    Firing,
    Timeout,
    Abort,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RequestedServoAction {
    Hold,
    MoveTo(i16),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ServoDispatch {
    pub action: RequestedServoAction,
    pub signal_angle_x10: Option<i16>,
}

impl ServoDispatch {
    const fn hold() -> Self {
        Self {
            action: RequestedServoAction::Hold,
            signal_angle_x10: None,
        }
    }

    const fn command(angle_x10: i16, signal: bool) -> Self {
        Self {
            action: RequestedServoAction::MoveTo(angle_x10),
            signal_angle_x10: if signal { Some(angle_x10) } else { None },
        }
    }
}

pub(crate) struct ServoCommandState {
    open_latched: bool,
    firing_open_send_count: u8,
    last_firing_open_send_ms: Option<u64>,
    open_latch_release_started_ms: Option<u64>,
    last_close_send_ms: Option<u64>,
    manual_open_command_active: bool,
}

impl ServoCommandState {
    pub const fn new() -> Self {
        Self {
            open_latched: false,
            firing_open_send_count: 0,
            last_firing_open_send_ms: None,
            open_latch_release_started_ms: None,
            last_close_send_ms: None,
            manual_open_command_active: false,
        }
    }

    pub fn update_latch_release(&mut self, now_ms: u64, release_delay_ms: u64) {
        if self.open_latched
            && self
                .open_latch_release_started_ms
                .is_some_and(|started| now_ms.saturating_sub(started) >= release_delay_ms)
        {
            self.open_latched = false;
            self.open_latch_release_started_ms = None;
            self.firing_open_send_count = 0;
            self.last_firing_open_send_ms = None;
        }
    }

    pub fn on_phase_transition(&mut self, phase: ServoPhase, now_ms: u64) {
        match phase {
            ServoPhase::Firing if !self.open_latched => {
                self.firing_open_send_count = 0;
                self.last_firing_open_send_ms = None;
                self.open_latch_release_started_ms = None;
            }
            ServoPhase::Timeout | ServoPhase::Abort if self.open_latched => {
                if self.open_latch_release_started_ms.is_none() {
                    self.open_latch_release_started_ms = Some(now_ms);
                }
            }
            _ => {}
        }
    }

    pub const fn can_start_firing(&self) -> bool {
        !self.open_latched
    }

    pub const fn open_latched(&self) -> bool {
        self.open_latched
    }

    pub const fn firing_open_send_count(&self) -> u8 {
        self.firing_open_send_count
    }

    pub const fn release_pending(&self) -> bool {
        self.open_latch_release_started_ms.is_some()
    }

    #[allow(clippy::too_many_arguments)]
    pub fn schedule(
        &mut self,
        phase: ServoPhase,
        firing_open_phase: bool,
        requested: RequestedServoAction,
        now_ms: u64,
        poll_interval_ms: u64,
        max_open_attempts: u8,
        open_angle_x10: i16,
        closed_angle_x10: i16,
    ) -> ServoDispatch {
        if phase == ServoPhase::Firing && firing_open_phase {
            self.open_latched = true;
        }

        if self.open_latched {
            self.last_close_send_ms = None;
            self.manual_open_command_active = false;

            if phase != ServoPhase::Firing
                || !firing_open_phase
                || requested != RequestedServoAction::MoveTo(open_angle_x10)
            {
                return ServoDispatch::hold();
            }

            if self.firing_open_send_count >= max_open_attempts {
                let final_attempt_settled = self
                    .last_firing_open_send_ms
                    .is_some_and(|last| now_ms.saturating_sub(last) >= poll_interval_ms);
                return if final_attempt_settled {
                    ServoDispatch::hold()
                } else {
                    ServoDispatch::command(open_angle_x10, false)
                };
            }

            let due = self
                .last_firing_open_send_ms
                .is_none_or(|last| now_ms.saturating_sub(last) >= poll_interval_ms);
            if due {
                self.last_firing_open_send_ms = Some(now_ms);
                self.firing_open_send_count += 1;
            }
            return ServoDispatch::command(open_angle_x10, due);
        }

        if phase == ServoPhase::Firing {
            self.last_close_send_ms = None;
            self.manual_open_command_active = false;
            return ServoDispatch::hold();
        }

        match requested {
            RequestedServoAction::MoveTo(angle) if angle == closed_angle_x10 => {
                self.manual_open_command_active = false;
                let due = self
                    .last_close_send_ms
                    .is_none_or(|last| now_ms.saturating_sub(last) >= poll_interval_ms);
                if due {
                    self.last_close_send_ms = Some(now_ms);
                }
                ServoDispatch::command(angle, due)
            }
            RequestedServoAction::MoveTo(angle) if angle == open_angle_x10 => {
                self.last_close_send_ms = None;
                let signal = !self.manual_open_command_active;
                self.manual_open_command_active = true;
                ServoDispatch::command(angle, signal)
            }
            RequestedServoAction::MoveTo(angle) => {
                self.last_close_send_ms = None;
                self.manual_open_command_active = false;
                ServoDispatch::command(angle, true)
            }
            RequestedServoAction::Hold => {
                self.last_close_send_ms = None;
                self.manual_open_command_active = false;
                ServoDispatch::hold()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OPEN: i16 = -348;
    const CLOSED: i16 = 552;
    const POLL_MS: u64 = 200;
    const MAX_OPEN: u8 = 10;

    fn schedule_open(state: &mut ServoCommandState, now_ms: u64) -> ServoDispatch {
        state.schedule(
            ServoPhase::Firing,
            true,
            RequestedServoAction::MoveTo(OPEN),
            now_ms,
            POLL_MS,
            MAX_OPEN,
            OPEN,
            CLOSED,
        )
    }

    #[test]
    fn firing_open_is_spaced_and_stops_after_ten_attempts() {
        let mut state = ServoCommandState::new();
        state.on_phase_transition(ServoPhase::Firing, 0);

        assert_eq!(
            schedule_open(&mut state, 13_000).signal_angle_x10,
            Some(OPEN)
        );
        assert!(state.open_latched());
        assert_eq!(schedule_open(&mut state, 13_199).signal_angle_x10, None);

        for attempt in 2..=MAX_OPEN {
            let now = 13_000 + u64::from(attempt - 1) * POLL_MS;
            assert_eq!(schedule_open(&mut state, now).signal_angle_x10, Some(OPEN));
        }

        assert_eq!(state.firing_open_send_count(), MAX_OPEN);
        assert_eq!(
            schedule_open(&mut state, 14_999),
            ServoDispatch::command(OPEN, false)
        );
        let dispatch = schedule_open(&mut state, 15_000);
        assert_eq!(dispatch, ServoDispatch::hold());
    }

    #[test]
    fn close_is_periodic_and_never_runs_during_firing() {
        let mut state = ServoCommandState::new();
        let requested = RequestedServoAction::MoveTo(CLOSED);

        let first = state.schedule(
            ServoPhase::Idle,
            false,
            requested,
            0,
            POLL_MS,
            MAX_OPEN,
            OPEN,
            CLOSED,
        );
        assert_eq!(first.signal_angle_x10, Some(CLOSED));
        assert_eq!(
            state
                .schedule(
                    ServoPhase::Idle,
                    false,
                    requested,
                    199,
                    POLL_MS,
                    MAX_OPEN,
                    OPEN,
                    CLOSED
                )
                .signal_angle_x10,
            None
        );
        assert_eq!(
            state
                .schedule(
                    ServoPhase::Idle,
                    false,
                    requested,
                    200,
                    POLL_MS,
                    MAX_OPEN,
                    OPEN,
                    CLOSED
                )
                .signal_angle_x10,
            Some(CLOSED)
        );
        assert_eq!(
            state.schedule(
                ServoPhase::Firing,
                false,
                requested,
                400,
                POLL_MS,
                MAX_OPEN,
                OPEN,
                CLOSED
            ),
            ServoDispatch::hold()
        );
    }

    #[test]
    fn latch_survives_abort_and_idle_until_release_delay() {
        let mut state = ServoCommandState::new();
        schedule_open(&mut state, 13_000);
        state.on_phase_transition(ServoPhase::Abort, 14_000);
        assert!(state.release_pending());

        state.on_phase_transition(ServoPhase::Idle, 14_100);
        state.update_latch_release(23_999, 10_000);
        assert!(state.open_latched());
        assert!(!state.can_start_firing());
        assert_eq!(
            state.schedule(
                ServoPhase::Idle,
                false,
                RequestedServoAction::MoveTo(CLOSED),
                23_999,
                POLL_MS,
                MAX_OPEN,
                OPEN,
                CLOSED,
            ),
            ServoDispatch::hold()
        );

        state.update_latch_release(24_000, 10_000);
        assert!(!state.open_latched());
        assert!(state.can_start_firing());
        assert_eq!(
            state
                .schedule(
                    ServoPhase::Idle,
                    false,
                    RequestedServoAction::MoveTo(CLOSED),
                    24_000,
                    POLL_MS,
                    MAX_OPEN,
                    OPEN,
                    CLOSED,
                )
                .signal_angle_x10,
            Some(CLOSED)
        );
    }

    #[test]
    fn timeout_starts_release_and_latched_state_forces_hold() {
        let mut state = ServoCommandState::new();
        schedule_open(&mut state, 13_000);
        state.on_phase_transition(ServoPhase::Timeout, 23_000);

        assert_eq!(
            state.schedule(
                ServoPhase::Timeout,
                false,
                RequestedServoAction::MoveTo(CLOSED),
                23_000,
                POLL_MS,
                MAX_OPEN,
                OPEN,
                CLOSED,
            ),
            ServoDispatch::hold()
        );
    }

    #[test]
    fn servo_inhibit_does_not_emit_or_count_an_open_attempt() {
        let mut state = ServoCommandState::new();
        assert_eq!(
            state.schedule(
                ServoPhase::Firing,
                true,
                RequestedServoAction::Hold,
                13_000,
                POLL_MS,
                MAX_OPEN,
                OPEN,
                CLOSED,
            ),
            ServoDispatch::hold()
        );
        assert!(state.open_latched());
        assert_eq!(state.firing_open_send_count(), 0);
    }
}
