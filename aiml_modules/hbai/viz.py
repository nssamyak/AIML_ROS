"""Plotting helpers that work on Linux, macOS, Windows and Raspberry Pi,
with a screen (windows open) or without one (figures saved to output/).
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np

from hbai.robot import RADIUS
from hbai.world import box_corners

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'output'


def parser(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument('--save', action='store_true', help='also save figures / GIFs to output/<module>/')
    p.add_argument('--no-show', action='store_true', help="don't open windows (implies --save)")
    p.add_argument('--fast', action='store_true', help='smaller, quicker run (used by run_all.py)')
    p.add_argument('--seed', type=int, default=0, help='random seed')
    return p


def _has_display():
    if sys.platform.startswith('linux') or 'bsd' in sys.platform:
        return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    return True     # Windows and macOS always have a window system


def init(p, module):
    """Parse arguments and pick a matplotlib backend. Returns (args, pyplot)."""
    args = p.parse_args()
    import matplotlib
    args.show = not args.no_show and _has_display()
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if args.show and matplotlib.get_backend().lower() == 'agg':
        print('matplotlib has no window backend here (on Linux: sudo apt install python3-tk). '
              'Saving figures instead.')
        args.show = False
    if not args.show and not args.no_show:
        print('No screen available: figures will be saved to output/ instead of shown.')
    args.save = args.save or not args.show
    args.out = OUTPUT / module
    if args.save:
        args.out.mkdir(parents=True, exist_ok=True)
    return args, plt


def finish(fig, args, name):
    """Save the figure (if saving)."""
    if args.save:
        path = args.out / f'{name}.png'
        fig.savefig(path, dpi=110, bbox_inches='tight')
        print(f'saved {path.relative_to(ROOT)}')


def show(args):
    if args.show:
        import matplotlib.pyplot as plt
        plt.show()


class Animator:
    """Show a loop live in a window and/or record it as a GIF."""

    def __init__(self, fig, args, name, fps=10):
        import matplotlib.pyplot as plt
        self.plt, self.fig, self.args = plt, fig, args
        self.writer = None
        self.live = args.show
        if args.save:
            from matplotlib.animation import PillowWriter
            self.path = args.out / f'{name}.gif'
            self.writer = PillowWriter(fps=fps)
            self.writer.setup(fig, str(self.path), dpi=70)
        if self.live:
            plt.ion()
            plt.show(block=False)

    def frame(self):
        if self.live:
            if not self.plt.fignum_exists(self.fig.number):
                self.live = False             # window closed: keep computing, stop drawing
            else:
                self.fig.canvas.draw_idle()
                self.plt.pause(0.001)
        if self.writer is not None:
            self.writer.grab_frame()

    def close(self):
        if self.writer is not None:
            self.writer.finish()
            print(f'saved {self.path.relative_to(ROOT)}')
        if self.args.show:
            self.plt.ioff()


# ---------------------------------------------------------------------- drawing
def draw_world(ax, world, color='0.3', alpha=1.0, zorder=1):
    from matplotlib.patches import Circle, Polygon
    for b in world.boxes:
        ax.add_patch(Polygon(box_corners(*b), closed=True, fc=color, ec='none', alpha=alpha, zorder=zorder))
    for cx, cy, r in world.circles:
        ax.add_patch(Circle((cx, cy), r, fc=color, ec='none', alpha=alpha, zorder=zorder))
    frame(ax)


def frame(ax):
    ax.set_xlim(-2.15, 2.15)
    ax.set_ylim(-1.65, 1.65)
    ax.set_aspect('equal')


def draw_robot(ax, pose, color='tab:blue', radius=RADIUS, zorder=5):
    from matplotlib.patches import Circle
    x, y, th = pose
    ax.add_patch(Circle((x, y), radius, fc=color, ec='k', lw=0.8, alpha=0.9, zorder=zorder))
    ax.plot([x, x + radius * np.cos(th)], [y, y + radius * np.sin(th)], 'w-', lw=1.5, zorder=zorder + 1)


def map_image(values):
    """ROS-style grid (-1 unknown, 0..100) -> grey image: unknown mid-grey, free white, walls black."""
    img = 1.0 - values.astype(float) / 100.0
    img[values < 0] = 0.78
    return img


def show_map(ax, values, geom, **kw):
    return ax.imshow(map_image(values), origin='lower', extent=geom.extent, cmap='gray',
                     vmin=0, vmax=1, interpolation='nearest', **kw)
