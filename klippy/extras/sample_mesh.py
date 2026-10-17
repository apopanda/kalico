# Sample focus-surface mesh for the microscope stage.
#
# The OFM server measures the sample's focus surface with its smooth
# autofocus engine (shape-gated sweeps) and hands the resulting grid
# here. The grid is installed through bed_mesh's own ZMesh machinery,
# so every gcode-layer move gets surface compensation from the stock
# transform. Jog moves bypass the gcode layer (dedicated trapq), so
# this module also wraps the jog executor to add the surface delta
# along XY moves while a mesh is active.
#
# No probe hardware is involved: values are focus heights, positive
# means the surface sits higher (physical z = gcode z + calc_z),
# matching bed_mesh's transform sign convention.
#
# Config:
#   [sample_mesh]
#   max_z_range: peak-to-peak rejection threshold, z units (default 512)
#   max_cell_jump: rejection threshold between adjacent grid points (default 150)
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
        self._zmesh = None
        self._follow = False
        self._grid = None
        self._orig_execute = None
        self._wrapped_jog = None
        self.gcode.register_command(
            'SAMPLE_MESH', self.cmd_SAMPLE_MESH, desc=self.cmd_SAMPLE_MESH_help)
        self.gcode.register_command(
            'SAMPLE_MESH_CLEAR', self.cmd_SAMPLE_MESH_CLEAR,
            desc=self.cmd_SAMPLE_MESH_CLEAR_help)
        self.gcode.register_command(
            'SAMPLE_MESH_STATUS', self.cmd_SAMPLE_MESH_STATUS,
            desc=self.cmd_SAMPLE_MESH_STATUS_help)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'klippy:disconnect', self._handle_disconnect)

    # ------------------------------------------------------------------
    # jog surface-follow

    def _handle_ready(self):
        jog = self.printer.lookup_object('jog', None)
        if jog is None or self._wrapped_jog is not None:
            return
        self._orig_execute = jog._execute_move
        jog._execute_move = self._execute_move_with_mesh
        self._wrapped_jog = jog
        logging.info('sample_mesh: jog surface-follow armed')

    def _handle_disconnect(self):
        if self._wrapped_jog is not None:
            self._wrapped_jog._execute_move = self._orig_execute
            self._wrapped_jog = None
            self._orig_execute = None

    def _follow_zmesh(self):
        if self._zmesh is not None and self._follow:
            return self._zmesh
        return None

    def _execute_move_with_mesh(self, speed, dist=[], accel=0.):
        mesh = self._follow_zmesh()
        dist = list(dist)
        while len(dist) < 3:
            dist.append(0.)
        if mesh is None or (abs(dist[0]) < 1e-12 and abs(dist[1]) < 1e-12):
            self._orig_execute(speed, dist, accel)
            return
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        x1 = pos[0] + dist[0]
        y1 = pos[1] + dist[1]
        dz = mesh.calc_z(x1, y1) - mesh.calc_z(pos[0], pos[1])
        dist[2] += dz
        logging.info(
            'sample_mesh: jog follow dz=%.4f (%.2f,%.2f)->(%.2f,%.2f)',
            dz, pos[0], pos[1], x1, y1)
        self._orig_execute(speed, dist, accel)

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
        mesh = ZMesh(params, 'sample')
        mesh.build_mesh(matrix)
        self._install(bedmesh, mesh, follow=follow, grid=matrix)
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
        self._follow = False
        self._grid = None
        gcmd.respond_info('sample_mesh: cleared')

    cmd_SAMPLE_MESH_STATUS_help = 'Report sample mesh state'

    def cmd_SAMPLE_MESH_STATUS(self, gcmd):
        if self._zmesh is None:
            gcmd.respond_info('sample_mesh: no mesh active')
            return
        zmin, zmax = self._zmesh.get_z_range()
        gcmd.respond_info(
            'sample_mesh: profile=%s bounds=(%.1f,%.1f)-(%.1f,%.1f) '
            'z range %.2f..%.2f follow=%s'
            % (self._zmesh.get_profile_name(),
               self._zmesh.mesh_x_min, self._zmesh.mesh_y_min,
               self._zmesh.mesh_x_max, self._zmesh.mesh_y_max,
               zmin, zmax, self._follow))

    def _install(self, bedmesh, mesh, follow=False, grid=None):
        self._zmesh = None
        self._follow = False
        self._grid = None
        if mesh is None:
            bedmesh.set_mesh(None)
            return
        self._zmesh = mesh
        self._follow = follow
        self._grid = grid
        bedmesh.set_mesh(mesh)


def load_config(config):
    return SampleMesh(config)
