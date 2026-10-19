# Jog support: drip-pattern segment release for low-latency manual moves.
#
# All jog motion logic lives here so upstream toolhead/force_move changes
# merge cleanly. toolhead delegates jog motion into this module.
import logging
import math

from klippy import chelper

DRIP_SEGMENT_TIME = 0.050
DRIP_TIME = 0.350
STEPCOMPRESS_FLUSH_TIME = 0.050


class JogInterrupted(Exception):
    pass


class JogMotion:
    def __init__(self, printer, jog):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self._jog = jog
        self._interrupted = False
        self._worker_running = False
        self._queue = []

    # Called from the interrupt path (webhook thread, mutex-free):
    # physical stop must never wait on the handshake.
    def request_interrupt(self):
        self._interrupted = True

    def is_interrupted(self):
        return self._interrupted

    def wait_idle(self, timeout=2.0):
        deadline = self.reactor.monotonic() + timeout
        while (
                self._queue or self._worker_running
        ) and self.reactor.monotonic() < deadline:
            self.reactor.pause(self.reactor.monotonic() + 0.005)

    def _check_interrupt(self):
        if self._interrupted:
            raise JogInterrupted()

    # Queue a move and ensure a single worker callback owns it. The worker
    # (reactor callback) is the only mutator of the jog clock: it schedules
    # the move into the jog trapq and then drip-releases it. JOG_MOVE's
    # gcode dispatch returns immediately, so the request channel frees and
    # a key-up JOG_INTERRUPT can land while the move is still releasing.
    def enqueue_move(self, speed, dist, accel):
        self._queue.append((speed, dist, accel))
        if not self._worker_running:
            self._worker_running = True
            self.reactor.register_callback(
                lambda e, s=self: s._release_worker()
            )

    def _release_worker(self):
        self._interrupted = False
        try:
            while self._queue:
                self._check_interrupt()
                speed, dist, accel = self._queue.pop(0)
                self._jog._execute_move(speed, dist, accel)
                if self._queue and not self._interrupted:
                    self.reactor.pause(self.reactor.monotonic())
        except JogInterrupted:
            logging.info("JOGDIAG: move abandoned on interrupt")
            self._queue = []
            self._resync_toolhead_clock()
        finally:
            self._worker_running = False

    def _resync_toolhead_clock(self):
        toolhead = self.printer.lookup_object('toolhead')
        est = toolhead.mcu.estimated_print_time(
            self.reactor.monotonic()
        )
        if toolhead.print_time > est:
            toolhead.print_time = est
            toolhead.min_restart_time = 0.


class Jog:
    def __init__(self, config):
        self.printer = config.get_printer()
        self._speed = 0.
        self._acceleration = 0.
        self._distance_vector = [0., 0., 0.]
        self._prev_jog_time = 0.
        self._jog_base_toolhead = None
        self._jog_base_kinfo = None
        self._motion = JogMotion(self.printer, self)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.stepper_kinematics = {
            'x': ffi_main.gc(
                ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free),
            'y': ffi_main.gc(
                ffi_lib.cartesian_stepper_alloc(b'y'), ffi_lib.free),
            'z': ffi_main.gc(
                ffi_lib.cartesian_stepper_alloc(b'z'), ffi_lib.free)
        }
        self._jog_swapped = False

    # implement default acceleration / speed taken from stepper

    def set_speed(self, speed):
        self._speed = speed

    def set_acceleration(self, acceleration):
        self._acceleration = acceleration

    def set_distance_vector(self, dist):
        self._distance_vector = dist

    def set_prev_jog_time(self, prev_jog_time):
        self._prev_jog_time = prev_jog_time

    def reset(self):
        self._speed = 0.
        self._acceleration = 0.
        self._distance_vector = [0., 0., 0.]

    def get_speed(self):
        return self._speed

    def get_acceleration(self):
        return self._acceleration

    def get_distance_vector(self):
        return self._distance_vector

    def get_prev_jog_time(self):
        return self._prev_jog_time

    def get_motion(self):
        return self._motion

    def manual_move_multiple(self, speed, dist=[], accel=0.):
        self._motion.enqueue_move(speed, dist, accel)

    def _execute_move(self, speed, dist=[], accel=0.):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        steppers = toolhead.get_kinematics().get_steppers()
        axis_map = {'X': 0, 'Y': 1, 'Z': 2}

        if not self._jog_swapped:
            for stepper in steppers:
                for axis, pos in axis_map.items():
                    if stepper.is_active_axis(axis.lower()):
                        stepper.set_stepper_kinematics(self.stepper_kinematics[axis.lower()])
                stepper.set_trapq(self.trapq)
            self._jog_swapped = True
            self._jog_base_toolhead = list(toolhead.get_position()[:3])
            cinfob = [(s.get_name(), s.get_mcu_position() * s._step_dist)
                      for s in steppers]
            self._jog_base_kinfo = list(
                toolhead.get_kinematics().calc_position(dict(cinfob)))[:3]
            self.set_toolhead_pos_from_steppers(steppers, toolhead.get_last_move_time())

        cp = toolhead.get_position()

        axis_r, accel_t, cruise_t, cruise_v = calc_moves_time(speed, accel, dist)
        print_time = max(toolhead.get_last_move_time(), self._prev_jog_time)
        logging.info(
            "MOVEDIAG: est=%.3f sched=%.3f dur=%.3f dist=%s cp=%s",
            toolhead.mcu.estimated_print_time(toolhead.reactor.monotonic()),
            print_time, accel_t + cruise_t + accel_t,
            [round(d, 2) for d in dist[:3]], [round(v, 2) for v in cp[:3]])
        self.trapq_append(self.trapq, print_time, accel_t, cruise_t, accel_t,
                          cp[axis_map['X']],
                          cp[axis_map['Y']],
                          cp[axis_map['Z']],
                          axis_r[axis_map['X']],
                          axis_r[axis_map['Y']],
                          axis_r[axis_map['Z']],
                          0., cruise_v, accel)
        end_time = print_time + accel_t + cruise_t + accel_t
        for stepper in steppers:
            for axis, pos in axis_map.items():
                if stepper.is_active_axis(axis.lower()):
                    stepper.generate_steps(end_time)
        self.trapq_finalize_moves(
            self.trapq, end_time + 99999.9, end_time + 99999.9
        )
        toolhead.note_mcu_movequeue_activity(end_time)
        toolhead.flush_step_generation()
        self.set_prev_jog_time(end_time)
        self._drain_segments(toolhead, end_time)
        self.set_toolhead_pos_from_steppers(steppers, toolhead.get_last_move_time())

    def _drain_segments(self, toolhead, move_end):
        motion = self._motion
        reactor = motion.reactor
        flush_delay = (
                DRIP_TIME + STEPCOMPRESS_FLUSH_TIME + toolhead.kin_flush_delay
        )
        while toolhead.print_time < move_end:
            motion._check_interrupt()
            curtime = reactor.monotonic()
            est_print_time = toolhead.mcu.estimated_print_time(curtime)
            wait_time = (
                    toolhead.print_time - est_print_time - flush_delay
            )
            if wait_time > 0.0:
                reactor.pause(curtime + min(wait_time, 0.005))
                continue
            npt = min(
                toolhead.print_time + DRIP_SEGMENT_TIME, move_end
            )
            toolhead.note_mcu_movequeue_activity(
                npt + toolhead.kin_flush_delay, set_step_gen_time=True
            )
            toolhead._advance_move_time(npt)

    def set_toolhead_pos_from_steppers(self, steppers, previous_jog_time):
        base = getattr(self, '_jog_base_kinfo', None)
        if base is None:
            return
        toolhead = self.printer.lookup_object('toolhead')
        cinfo = [(s.get_name(), s.get_mcu_position() * s._step_dist) for s in
                 steppers]
        kinfo = list(toolhead.get_kinematics().calc_position(dict(cinfo)))
        for i in range(3):
            kinfo[i] = self._jog_base_toolhead[i] + (
                    kinfo[i] - base[i])
        kinfo.append(0.)  # todo handle extrusion stepper
        toolhead.set_position(kinfo)

    def note_jogging_end(self):
        toolhead = self.printer.lookup_object('toolhead')
        last_jog_time = self._prev_jog_time
        steppers = toolhead.get_kinematics().get_steppers()
        for stepper in steppers:
            stepper.note_homing_end()
        self.set_toolhead_pos_from_steppers(steppers, last_jog_time)

    def get_jog_trapq(self):
        return self.trapq

    def jog_restore(self):
        toolhead = self.printer.lookup_object('toolhead')
        steppers = toolhead.get_kinematics().get_steppers()
        trapq = toolhead.get_trapq()
        for stepper in steppers:
            sk = stepper.get_pre_jog_kinematics()
            if sk is not None:
                stepper.set_stepper_kinematics(sk)
            stepper.set_trapq(trapq)
            stepper.reset_pre_jog_kinematics()
            stepper.reset_pre_jog_trapq()
        self._jog_swapped = False
        self.set_toolhead_pos_from_steppers(steppers, toolhead.get_last_move_time())
        self._jog_base_toolhead = None
        self._jog_base_kinfo = None


def normalize(distances=None):
    if distances is None:
        distances = []
    magnitude = magnitude_v(distances)
    if magnitude == 0.:
        magnitude = 1.
    return list(map(lambda dist: dist / magnitude, distances))


def magnitude_v(distances):
    return math.sqrt(sum([d * d for d in distances[:3]]))


def calc_moves_time(speed, accel, dist=None):
    if dist is None:
        dist = [0., 0., 0.]
    axis_r = normalize(dist)
    distances = list(map(lambda distance: abs(distance), dist))
    magnitude = magnitude_v(distances)

    if not accel or not magnitude:
        return axis_r, 0., magnitude / speed, speed
    max_cruise_v2 = magnitude * accel
    if max_cruise_v2 < speed ** 2:
        speed = math.sqrt(max_cruise_v2)
    accel_t = speed / accel
    accel_decel_d = accel_t * speed
    cruise_t = abs((magnitude - accel_decel_d) / speed)
    return axis_r, accel_t, cruise_t, speed


def load_config(config):
    return Jog(config)
