from tkinter import ttk
import tkinter as tk


class AutoScrollbar(ttk.Scrollbar):
    """A scrollbar that hides itself if it's not needed.
        Works only if you use the grid geometry manager """
    def set(self, lo, hi):
        if float(lo) <= 0.0 and float(hi) >= 1.0:
            self.grid_remove()
        else:
            self.grid()
            ttk.Scrollbar.set(self, lo, hi)

    def pack(self, **kw):
        raise tk.TclError("Cannot use pack with this widget")

    def place(self, **kw):
        raise tk.TclError("Cannot use place with this widget")


class Player:
    def __init__(self, scale, frame_count):
        self._playing = False
        self._paused = 0
        self._play_speed = 100

        self.master = tk.Tk()

        # Vertical and horizontal scrollbars for canvas
        vbar = AutoScrollbar(self.master, orient="vertical")
        hbar = AutoScrollbar(self.master, orient="horizontal")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="we")

        self.timeline = tk.Scale(self.master, from_=0, to=frame_count, orient=tk.HORIZONTAL)
        self.timeline.grid(row=2, column=0, sticky="nswe")

        # Create canvas and put image on it
        self.canvas = tk.Canvas(
            self.master,
            highlightthickness=0,
            xscrollcommand=hbar.set,
            yscrollcommand=vbar.set
        )
        self.canvas.grid(row=0, column=0, sticky='nswe')
        self.canvas.update()  # wait till canvas is created
        vbar.configure(command=self.canvas.yview)  # bind scrollbars to the canvas
        hbar.configure(command=self.canvas.xview)
        # Make the canvas expandable
        self.master.rowconfigure(0, weight=1)
        self.master.columnconfigure(0, weight=1)

        # Bind events to the Canvas
        self.canvas.bind('<ButtonPress-1>', self._move_from)
        self.canvas.bind('<B1-Motion>',     self._move_to)
        self.canvas.bind('<MouseWheel>',    self._wheel)

        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        self.scale = scale    # current scale
        self.delta = 1.3      # zoom

        this = self
        self.master.bind("<Right>", lambda key: this.timeline.set(this.timeline.get() + 1))
        self.master.bind("<Left>", lambda key: this.timeline.set(this.timeline.get() - 1))

        self.master.bind("<Up>", lambda key: this._modify_play_speed(25))
        self.master.bind("<Down>", lambda key: this._modify_play_speed(-25))

        self.master.bind("<space>", lambda key: this.toggle())
        self._tick()

    def _wheel(self, event):
        scale = 1
        if event.num == 5 or event.delta == -120:  # scroll down
            scale = 1 / self.delta
        elif event.num == 4 or event.delta == 120:  # scroll up
            scale = self.delta
        else:
            return

        self.scale *= scale
        self.canvas.scale("all", 0, 0, scale, scale)  # rescale all canvas objects
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _move_from(self, event):
        """Remember previous coordinates for scrolling with the mouse"""
        self.canvas.scan_mark(event.x, event.y)

    def _move_to(self, event):
        """Drag (move) canvas to the new position"""
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    def _tick(self, key=None):
        if self._playing:
            self.timeline.set(self.timeline.get() + 1)

        self.timeline.after(self._play_speed, self._tick)

    def _modify_play_speed(self, change):
        new_speed = self._play_speed + change
        self._play_speed = min(max(new_speed, 20), 1000)

    def main_loop(self):
        self.master.mainloop()

    def toggle(self, key=None):
        self._playing = not self._playing

    def set_redraw(self, redraw):
        self.timeline.config(command=redraw)
        pass

    def update_scroll(self):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def text(self, p, text, color="white"):
        x, y = p[0] * self.scale, p[1] * self.scale
        result = self.canvas.create_text(
            x, y,
            anchor=tk.NW,
            text=text,
            fill=color
        )
        return result

    def hide_item(self, item):
        self.canvas.itemconfigure(item, state='hidden')

    def show_item(self, item):
        self.canvas.itemconfigure(item, state='normal')

    def move_circle(self, item, target_pos):
        x, y = target_pos[0] * self.scale, target_pos[1] * self.scale
        cur = self.canvas.coords(item)
        rx, ry = (cur[2] - cur[0]) / 2, (cur[3] - cur[1]) / 2

        self.canvas.coords(item, x - rx, y - ry, x + rx, y + ry)

    def line(self, p1, p2, color="white", **kwargs):
        sp1 = p1[0] * self.scale, p1[1] * self.scale
        sp2 = p2[0] * self.scale, p2[1] * self.scale

        result = self.canvas.create_line(*sp1, *sp2, fill=color, **kwargs)

        return result

    def circle(self, p, R, fill="white", outline="white"):
        x, y = p[0] * self.scale, p[1] * self.scale
        r = R * self.scale
        result = self.canvas.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline=outline)

        return result

    def move_orientation(self, item, target):
        pos1 = (target.pos[0] + target.orient[0] * target.R) * self.scale, (target.pos[1] + target.orient[1] * target.R) * self.scale
        pos = target.pos[0] * self.scale, target.pos[1] * self.scale

        self.canvas.coords(item, *pos, *pos1)

    def orientation(self, agt, color="white"):
        pos1 = agt.pos[0] + agt.orient[0] * agt.R, agt.pos[1] + agt.orient[1] * agt.R
        return self.line(agt.pos, pos1, color)

    def vector(self, p1, p2, color="white"):
        self.line(p1, p2, color)

        p09 = p1[0] + (p2[0] - p1[0]) * 0.9, p1[1] + (p2[1] - p1[1]) * 0.9
        orto01 = -(p2[1] - p1[1]) * 0.05, (p2[0] - p1[0]) * 0.05

        left_wing = p09[0] + orto01[0], p09[1] + orto01[1]
        right_wing = p09[0] - orto01[0], p09[1] - orto01[1]

        self.line(left_wing,  p2, color)
        self.line(right_wing, p2, color)
        self.line(right_wing, left_wing, color)
