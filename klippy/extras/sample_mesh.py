# Sample focus-surface mesh for the microscope stage.
#
# The OFM server measures the sample's focus surface with its smooth
# autofocus engine (shape-gated sweeps) and hands the resulting grid
# here. The grid is installed through bed_mesh's own ZMesh machinery,
# so every gcode-layer move gets surface compensation from the stock
# transform. User jogs bypass the gcode layer (dedicated trapq), so
# this module also hooks the jog move queue to add the surface delta
# along XY jogs while a mesh is active. Automation never jogs: all
# server-side motion is planned G-code and is compensated by the
# bed_mesh transform alone.
#
# No probe hardware is involved: values are focus heights, positive
# means the surface sits higher (physical z = gcode z + calc_z),
# matching bed_mesh's transform sign convention.
#
# Config:
#   [sample_mesh]
#   max_z_range: peak-to-peak rejection threshold, z units (default 512)
#   max_cell_jump: rejection threshold between adjacent grid points (default 150)
#   jog_follow: hook the user jog queue for surface following (default false)
#
# Coordinates and z values are stage units exactly as klippy sees them
# (OFM raw units map numerically 1:1 to klippy internals).

import logging

from .bed_mesh import ZMesh


class SampleMesh:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.max_z_range = config.getfloat('max_z_range', 512.0, above=0.)
        self.max_cell_jump = config.getfloat('max_cell_jump', 150.0, above=0.)
        self.jog_follow = config.getboolean('jog_follow', False)
        self._zmesh = None
        self._follow = False
        self._grid = None
        self._orig_enqueue = None
        self._wrapped_motion = None
        self._hook_xy = None
        self._orig_skp = None
        self._last_params = None
        self._anchor_z = 0.0
        self._suspended = False
        self.gcode.register_command(
            'SAMPLE_MESH', self.cmd_SAMPLE_MESH, desc=self.cmd_SAMPLE_MESH_help)
        self.gcode.register_command(
            'SAMPLE_MESH_CLEAR', self.cmd_SAMPLE_MESH_CLEAR,
            desc=self.cmd_SAMPLE_MESH_CLEAR_help)
        self.gcode.register_command(
            'SAMPLE_MESH_STATUS', self.cmd_SAMPLE_MESH_STATUS,
            desc=self.cmd_SAMPLE_MESH_STATUS_help)
        self.gcode.register_command(
            'SAMPLE_MESH_SHIFT', self.cmd_SAMPLE_MESH_SHIFT,
            desc=self.cmd_SAMPLE_MESH_SHIFT_help)
        self.gcode.register_command(
            'SAMPLE_MESH_REANCHOR', self.cmd_SAMPLE_MESH_REANCHOR,
            desc=self.cmd_SAMPLE_MESH_REANCHOR_help)
        self.gcode.register_command(
            'SAMPLE_MESH_SUSPEND', self.cmd_SAMPLE_MESH_SUSPEND,
            desc=self.cmd_SAMPLE_MESH_SUSPEND_help)
        self.gcode.register_command(
            'SAMPLE_MESH_RESUME', self.cmd_SAMPLE_MESH_RESUME,
            desc=self.cmd_SAMPLE_MESH_RESUME_help)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'klippy:disconnect', self._handle_disconnect)

    # ------------------------------------------------------------------
    # jog surface-follow (user jogs only; automation stays on G-code)

    def _handle_ready(self):
        jog = self.printer.lookup_object('jog', None)
        if jog is None or self._wrapped_motion is not None:
            return
        motion = getattr(jog, '_motion', None)
        if not self.jog_follow or motion is None:
            if motion is None and self.jog_follow:
                logging.info(
                    'sample_mesh: jog extra has no motion queue - '
                    'surface-follow unavailable')
            else:
                logging.info(
                    'sample_mesh: jog follow disabled by config '
                    '(stock jog path preserved)')
            self._arm_skp_guard()
            return
        self._orig_enqueue = motion.enqueue_move
        motion.enqueue_move = self._enqueue_move_with_mesh
        self._wrapped_motion = motion
        self._hook_xy = None
        logging.info('sample_mesh: jog surface-follow armed (enqueue hook)')
        self._arm_skp_guard()

    def _arm_skp_guard(self):
        fm = self.printer.lookup_object('force_move', None)
        if fm is not None:
            self._orig_skp = self.gcode.register_command(
                'SET_KINEMATIC_POSITION', None)
            self.gcode.register_command(
                'SET_KINEMATIC_POSITION', self._skp_clear_mesh)
            logging.info(
                'sample_mesh: SET_KINEMATIC_POSITION guard armed')

    def _skp_clear_mesh(self, gcmd):
        if self._orig_skp is not None:
            self._orig_skp(gcmd)
        rezeros = [gcmd.get(ax, None) is not None for ax in 'XYZ']
        if not any(rezeros):
            logging.info(
                'sample_mesh: SET_KINEMATIC_POSITION without X/Y/Z '
                '(homing flag update) - mesh kept')
            return
        if self._zmesh is not None:
            logging.info(
                'sample_mesh: frame re-zeroed, clearing stale mesh')
            bedmesh = self.printer.lookup_object('bed_mesh', None)
            if bedmesh is not None:
                bedmesh.set_mesh(None)
            self._zmesh = None
            self._follow = False
            self._grid = None
        self._hook_xy = None

    def _handle_disconnect(self):
        if self._wrapped_motion is not None:
            self._wrapped_motion.enqueue_move = self._orig_enqueue
            self._wrapped_motion = None
            self._orig_enqueue = None

    def _follow_zmesh(self):
        if self._suspended:
            return None
        if self._zmesh is not None and self._follow:
            return self._zmesh
        return None

    JOG_SEG_LEN = 100.0
    JOG_SEG_DZ = 15.0

    def _jog_segments(self, mesh, x0, y0, dist):
        total = (dist[0] * dist[0] + dist[1] * dist[1]) ** 0.5
        if total < 1e-9:
            return []
        ux, uy = dist[0] / total, dist[1] / total
        segs = []
        cx, cy = x0, y0
        remaining = total
        while remaining > 1e-9:
            L = remaining
            while L > self.JOG_SEG_LEN:
                dz = (mesh.calc_z(cx + ux * L, cy + uy * L)
                      - mesh.calc_z(cx, cy))
                if abs(dz) <= self.JOG_SEG_DZ:
                    break
                L = max(self.JOG_SEG_LEN, L / 2.0)
            dz = mesh.calc_z(cx + ux * L, cy + uy * L) - mesh.calc_z(cx, cy)
            segs.append((ux * L, uy * L, dz))
            cx += ux * L
            cy += uy * L
            remaining -= L
        return segs

    def _enqueue_move_with_mesh(self, speed, dist, accel=0.):
        motion = self._wrapped_motion
        mesh = self._follow_zmesh()
        dist = list(dist)
        while len(dist) < 3:
            dist.append(0.)
        if motion is None or mesh is None or (
                abs(dist[0]) < 1e-12 and abs(dist[1]) < 1e-12):
            self._orig_enqueue(speed, dist, accel)
            return
        if not (motion._queue or motion._worker_running):
            self._hook_xy = None
        if self._hook_xy is None:
            toolhead = self.printer.lookup_object('toolhead')
            pos = toolhead.get_position()
            self._hook_xy = (pos[0], pos[1])
        x0, y0 = self._hook_xy
        segs = self._jog_segments(mesh, x0, y0, dist)
        if len(segs) <= 1:
            dz = segs[0][2] if segs else 0.0
            dist[2] += dz
            logging.info(
                'sample_mesh: jog follow dz=%.4f (%.2f,%.2f)->(%.2f,%.2f)',
                dz, x0, y0, x0 + dist[0], y0 + dist[1])
            self._orig_enqueue(speed, dist, accel)
        else:
            for sx, sy, dz in segs:
                self._orig_enqueue(speed, [sx, sy, dz], accel)
            logging.info(
                'sample_mesh: %d adaptive segments (%.1f,%.1f) '
                'd=(%.1f,%.1f) dz_total=%.2f',
                len(segs), x0, y0, dist[0], dist[1],
                sum(s[2] for s in segs))
        self._hook_xy = (x0 + dist[0], y0 + dist[1])

    # ------------------------------------------------------------------
    # commands

    cmd_SAMPLE_MESH_help = 'Install a measured sample focus-surface mesh'

    def cmd_SAMPLE_MESH(self, gcmd):
        bedmesh = self.printer.lookup_object('bed_mesh', None)
        if bedmesh is None:
            raise gcmd.error('sample_mesh: [bed_mesh] is not loaded')
        cols = gcmd.get_int('COLS', 3, minval=2, maxval=15)
        rows = gcmd.get_int('ROWS', 3, minval=2, maxval=15)
        min_x = gcmd.get_float('MIN_X')
        max_x = gcmd.get_float('MAX_X')
        min_y = gcmd.get_float('MIN_Y')
        max_y = gcmd.get_float('MAX_Y')
        if max_x <= min_x or max_y <= min_y:
            raise gcmd.error('sample_mesh: empty bounds (MAX must exceed MIN)')
        algo = gcmd.get('ALGO', 'direct')
        if algo not in ('lagrange', 'bicubic', 'direct'):
            raise gcmd.error('sample_mesh: ALGO must be lagrange, bicubic or direct')
        pps = 0 if algo == 'direct' else 2
        follow = bool(gcmd.get_int('FOLLOW', 1, minval=0, maxval=1))
        raw = gcmd.get('GRID').split()
        if len(raw) != rows * cols:
            raise gcmd.error(
                'sample_mesh: GRID needs ROWS*COLS=%d values, got %d'
                % (rows * cols, len(raw)))
        try:
            values = [float(v) for v in raw]
        except ValueError:
            raise gcmd.error('sample_mesh: GRID values must be numbers')
        finite = all(v == v and abs(v) != float('inf') for v in values)
        if not finite:
            raise gcmd.error('sample_mesh: GRID contains non-finite values')
        matrix = [values[r * cols:(r + 1) * cols] for r in range(rows)]
        zmin = min(values)
        zmax = max(values)
        if zmax - zmin > self.max_z_range:
            self._install(bedmesh, None)
            raise gcmd.error(
                'sample_mesh: peak-to-peak range %.1f exceeds limit %.1f'
                % (zmax - zmin, self.max_z_range))
        for r in range(rows):
            for c in range(cols - 1):
                if abs(matrix[r][c + 1] - matrix[r][c]) > self.max_cell_jump:
                    self._install(bedmesh, None)
                    raise gcmd.error(
                        'sample_mesh: X-neighbor jump %.1f at row %d col %d '
                        'exceeds limit %.1f'
                        % (abs(matrix[r][c + 1] - matrix[r][c]), r, c,
                           self.max_cell_jump))
        for r in range(rows - 1):
            for c in range(cols):
                if abs(matrix[r + 1][c] - matrix[r][c]) > self.max_cell_jump:
                    self._install(bedmesh, None)
                    raise gcmd.error(
                        'sample_mesh: Y-neighbor jump %.1f at row %d col %d '
                        'exceeds limit %.1f'
                        % (abs(matrix[r + 1][c] - matrix[r][c]), r, c,
                           self.max_cell_jump))
        params = {
            'min_x': min_x,
            'max_x': max_x,
            'min_y': min_y,
            'max_y': max_y,
            'x_count': cols,
            'y_count': rows,
            'algo': algo,
            'mesh_x_pps': pps,
            'mesh_y_pps': pps,
        }
        self._last_params = dict(params)
        mesh = ZMesh(params, 'sample')
        mesh.build_mesh(matrix)
        self._install(bedmesh, mesh, follow=follow, grid=matrix)
        self._anchor_z = gcmd.get_float('ZREF', 0.0)
        self._hook_xy = None
        gcmd.respond_info(
            'sample_mesh: %dx%d grid active, bounds (%.1f,%.1f)-(%.1f,%.1f), '
            'z range %.2f..%.2f, follow=%s, algo=%s'
            % (cols, rows, min_x, min_y, max_x, max_y, zmin, zmax,
               follow, algo))

    cmd_SAMPLE_MESH_CLEAR_help = 'Remove the sample mesh and disable follow'

    def cmd_SAMPLE_MESH_CLEAR(self, gcmd):
        bedmesh = self.printer.lookup_object('bed_mesh', None)
        if bedmesh is not None:
            bedmesh.set_mesh(None)
        self._zmesh = None
        self._last_params = None
        self._follow = False
        self._grid = None
        self._hook_xy = None
        gcmd.respond_info('sample_mesh: cleared')

    cmd_SAMPLE_MESH_REANCHOR_help = (
        'Re-anchor the mesh plane to the current position (or Z=...) - '
        'run after jogging to true focus')

    def cmd_SAMPLE_MESH_REANCHOR(self, gcmd):
        if self._zmesh is None or self._grid is None:
            raise gcmd.error('sample_mesh: no mesh active to re-anchor')
        target = gcmd.get_float('Z', None)
        try:
            toolhead = self.printer.lookup_object('toolhead')
            pos = toolhead.get_position()
            surface = (
                self._anchor_z + self._zmesh.calc_z(pos[0], pos[1]))
        except Exception as e:
            raise gcmd.error(
                'sample_mesh: cannot read position: %s' % str(e))
        if target is None:
            target = pos[2]
        delta = target - surface
        try:
            zr = self._shift_mesh(delta)
        except RuntimeError as e:
            raise gcmd.error('sample_mesh: %s' % str(e))
        gcmd.respond_info(
            'sample_mesh: re-anchored plane to %.2f (grid %+.2f), '
            'z range %.2f..%.2f' % (target, delta, zr[0], zr[1]))

    cmd_SAMPLE_MESH_SHIFT_help = 'Shift the active sample mesh by a z delta'

    def _shift_mesh(self, delta):
        if self._zmesh is None or self._grid is None:
            raise RuntimeError('no mesh active to shift')
        if delta == 0:
            return None
        shift_max = self.max_z_range
        if abs(delta) > shift_max:
            raise RuntimeError(
                'shift %.1f exceeds sanity limit %.1f'
                % (abs(delta), shift_max))
        matrix = [[z + delta for z in row] for row in self._grid]
        zmin = min(min(row) for row in matrix)
        zmax = max(max(row) for row in matrix)
        params = {
            'min_x': self._zmesh.mesh_x_min,
            'max_x': self._zmesh.mesh_x_max,
            'min_y': self._zmesh.mesh_y_min,
            'max_y': self._zmesh.mesh_y_max,
            'x_count': len(matrix[0]),
            'y_count': len(matrix),
            'algo': self._last_params['algo'],
            'mesh_x_pps': self._last_params['mesh_x_pps'],
            'mesh_y_pps': self._last_params['mesh_y_pps'],
        }
        mesh = ZMesh(params, 'sample')
        mesh.build_mesh(matrix)
        bedmesh = self.printer.lookup_object('bed_mesh', None)
        anchor = self._anchor_z
        self._install(bedmesh, mesh, follow=self._follow, grid=matrix)
        self._anchor_z = anchor
        return zmin, zmax

    def cmd_SAMPLE_MESH_SHIFT(self, gcmd):
        delta = gcmd.get_float('DELTA')
        try:
            zr = self._shift_mesh(delta)
        except RuntimeError as e:
            raise gcmd.error('sample_mesh: %s' % str(e))
        if zr is None:
            gcmd.respond_info('sample_mesh: zero shift, mesh unchanged')
            return
        gcmd.respond_info(
            'sample_mesh: shifted by %.2f, z range %.2f..%.2f'
            % (delta, zr[0], zr[1]))

    cmd_SAMPLE_MESH_STATUS_help = 'Report sample mesh state'

    def cmd_SAMPLE_MESH_STATUS(self, gcmd):
        if self._zmesh is None:
            gcmd.respond_info('sample_mesh: no mesh active')
            return
        zmin, zmax = self._zmesh.get_z_range()
        try:
            toolhead = self.printer.lookup_object('toolhead')
            tpos = toolhead.get_position()
            surface = (
                self._anchor_z + self._zmesh.calc_z(tpos[0], tpos[1]))
            zref = tpos[2] - surface
        except Exception:
            zref = float('nan')
        gcmd.respond_info(
            'sample_mesh: profile=%s bounds=(%.1f,%.1f)-(%.1f,%.1f) '
            'z range %.2f..%.2f follow=%s zref=%.2f'
            % (self._zmesh.get_profile_name(),
               self._zmesh.mesh_x_min, self._zmesh.mesh_y_min,
               self._zmesh.mesh_x_max, self._zmesh.mesh_y_max,
               zmin, zmax, self._follow, zref))

    cmd_SAMPLE_MESH_SUSPEND_help = 'Temporarily disable the mesh transform'

    def cmd_SAMPLE_MESH_SUSPEND(self, gcmd):
        if self._zmesh is None:
            gcmd.respond_info('sample_mesh: no mesh active, nothing to suspend')
            return
        bedmesh = self.printer.lookup_object('bed_mesh', None)
        if bedmesh is not None:
            bedmesh.set_mesh(None)
        self._suspended = True
        logging.info('sample_mesh: transform suspended (measuring)')

    cmd_SAMPLE_MESH_RESUME_help = 'Re-enable the mesh transform'

    def cmd_SAMPLE_MESH_RESUME(self, gcmd):
        if not self._suspended:
            return
        bedmesh = self.printer.lookup_object('bed_mesh', None)
        if bedmesh is not None and self._zmesh is not None:
            bedmesh.set_mesh(self._zmesh)
        self._suspended = False
        logging.info('sample_mesh: transform resumed')

    def get_status(self, eventtime=None):
        if self._zmesh is None:
            return {
                "zref": None, "active": False, "follow": False,
                "surface_here": None,
            }
        zref = surface_here = None
        try:
            toolhead = self.printer.lookup_object('toolhead')
            pos = toolhead.get_position()
            surface_here = (
                self._anchor_z + self._zmesh.calc_z(pos[0], pos[1]))
            zref = pos[2] - surface_here
        except Exception:
            pass
        return {
            "zref": zref,
            "active": True,
            "follow": self._follow,
            "surface_here": surface_here,
        }

    def _install(self, bedmesh, mesh, follow=False, grid=None):
        self._zmesh = None
        self._follow = False
        self._grid = None
        self._anchor_z = 0.0
        if mesh is None:
            bedmesh.set_mesh(None)
            return
        if follow and not self.jog_follow:
            logging.info(
                'sample_mesh: FOLLOW=1 requested but jog_follow is '
                'disabled by config - installing scan-only mesh')
        self._zmesh = mesh
        self._follow = follow and self.jog_follow
        self._grid = grid
        bedmesh.set_mesh(mesh)


def load_config(config):
    return SampleMesh(config)
