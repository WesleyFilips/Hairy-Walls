#!/usr/bin/env python3
"""
hairy_walls_gui.py
===================
Hairy Walls -- a simple desktop GUI for hairy_walls.py: pick a .gcode
file, set an output name, fill in the parameters, and run -- no command
line needed.

Requires hairy_walls.py to be in the SAME FOLDER as this file (it's
imported directly and its process()/process_collision_aware() functions
are called in a background thread, so the window never freezes).

Run with:
    python hairy_walls_gui.py
"""

import os
import sys
import json
import threading
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

SETTINGS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hairy_walls_gui_settings.json')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import hairy_walls
except ImportError:
    root = tk.Tk()
    root.withdraw()
    messagebox.showerror(
        "hairy_walls.py not found",
        "This GUI needs hairy_walls.py in the same folder.\n\n"
        "Download it and place both files together, then run this again.")
    sys.exit(1)

import gcode3mf


# (attribute name on the args object, label text, kind, default, help text)
# kind is one of: float, int, optional_float, optional_int
FIELDS = [
    ('spacing', 'Spacing (mm)', 'float', 4.0,
     'Distance along the wall between the start of each loop.'),
    ('length', 'Extrude length (mm)', 'float', 1.5,
     'How far each loop travels outward WHILE extruding.'),
    ('dry_length', 'Dry length (mm)', 'float', 0.0,
     'Additional mm to keep traveling outward AFTER the extrude length, '
     'with no extrusion -- a dry "whip" tip. 0 = old behavior (no dry tip).'),
    ('wall_thickness', 'Wall thickness (mm)', 'float', 0.0,
     'Reference thickness that Wall overlap is a percent of. Set to your '
     'actual wall thickness (line width x wall count) to make Wall '
     'overlap meaningful.'),
    ('wall_overlap', 'Wall overlap (%)', 'float', 0.0,
     "How far inward from the wall's outer edge each hair's root starts, "
     'as a percent of Wall thickness: 0%% roots it right at the edge '
     '(default), 100%% roots it a full wall-thickness inward so the base '
     'is embedded in the wall. Outward length is unaffected.'),
    ('extrude', 'Extrude (mm)', 'float', 0.30,
     'Filament pushed out over the extrude length of each loop.'),
    ('root_extrude', 'Root extrude (mm)', 'float', 0.0,
     'Filament pushed out AT the wall, with no travel, before heading '
     'outward -- an anchor blob at the base of the hair. 0 = no root.'),
    ('return_extrude', 'Return extrude (mm)', 'float', 0.0,
     'Extra filament pushed out during the return travel (0 = none, so '
     'the return leg uses Dry feedrate).'),
    ('retract', 'Retract (mm)', 'float', 0.0,
     'Retract this much right after the extrude leg, before the dry '
     'travel and return -- cuts stringing. Automatically un-retracted by '
     'the same amount once back at the wall. 0 = no retract.'),
    ('retract_feedrate', 'Retract feedrate (mm/min)', 'float', 2100.0,
     'Speed for the retract and un-retract moves.'),
    ('extra_restart', 'Extra restart (mm)', 'float', 0.0,
     'Extra mm pushed during un-retract, on top of exactly restoring '
     'Retract -- helps rebuild melt pressure if the wall looks '
     'under-extruded right after each hair. Try 0.02-0.1.'),
    ('z_hop', 'Z-hop (mm)', 'float', 0.0,
     'Lift Z this much before traveling back from a hair tip to the wall '
     '(and between hairs in Hair plugs mode) so the nozzle clears what it '
     'just printed. 0 = no hop.'),
    ('z_hop_feedrate', 'Z-hop feedrate (mm/min)', 'float', 600.0,
     'Speed for the Z-hop up/down moves.'),
    ('feedrate', 'Extrude feedrate (mm/min)', 'float', 1200.0,
     'Speed for any move that is actively extruding: the outward extrude '
     'leg, and the return leg too if Return extrude is above 0.'),
    ('dry_feedrate', 'Dry feedrate (mm/min)', 'optional_float', None,
     'Speed for any move that is NOT extruding: the outward dry leg, and '
     'the return leg when Return extrude is 0. Leave blank to match '
     'Extrude feedrate.'),
    ('length_jitter', 'Length jitter (0-1)', 'float', 0.0,
     'Fractional random variation in loop length, e.g. 0.3 = +/-30%.'),
    ('angle_jitter', 'Angle jitter (deg)', 'float', 0.0,
     'Random rotation off perfectly perpendicular, for a less uniform look.'),
    ('seed', 'Random seed', 'int', 0,
     'Same seed always gives the same "random" pattern.'),
    ('min_contour_length', 'Min contour length (mm)', 'float', 8.0,
     'Skip outer-wall loops shorter than this (avoids tiny holes/text).'),
    ('min_z', 'Min Z (mm)', 'float', 0.0,
     'Only add loops at or above this height.'),
    ('max_z', 'Max Z (mm)', 'optional_float', None,
     'Only add loops at or below this height. Leave blank for no limit.'),
    ('collision_margin', 'Collision margin (mm)', 'float', 0.0,
     'Minimum clearance a loop must keep from other toolpaths. '
     'Only used when "Avoid collisions" is checked.'),
]

FUZZY_TOOL_SLOTS = 4


class Args:
    """Plain attribute bag -- hairy_walls.process() just needs attribute
    access, it doesn't care that this isn't a real argparse.Namespace."""
    pass


class HairyWallsGUI(ttk.Frame):
    def __init__(self, master):
        super().__init__(master, padding=12)
        self.master = master
        self.pack(fill='both', expand=True)

        self.input_path = tk.StringVar()
        self.output_path = tk.StringVar()
        self.random_phase = tk.BooleanVar(value=False)
        self.avoid_collisions = tk.BooleanVar(value=False)
        self.mode = tk.StringVar(value='wall-interrupt')
        self.field_vars = {}
        # one (tool_var, remap_var) pair per fuzzy-tool slot
        self.fuzzy_tool_vars = [(tk.StringVar(), tk.StringVar()) for _ in range(FUZZY_TOOL_SLOTS)]

        self._build_file_row()
        self._build_mode_row()
        self._build_fuzzy_tools_section()
        self._build_param_grid()
        self._build_flag_row()
        self._build_run_row()
        self._build_log()

        self._load_settings()

    # -- layout ------------------------------------------------------

    def _build_file_row(self):
        frame = ttk.LabelFrame(self, text="Files", padding=8)
        frame.pack(fill='x', pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Input .gcode:").grid(row=0, column=0, sticky='w', pady=2)
        ttk.Entry(frame, textvariable=self.input_path).grid(row=0, column=1, sticky='ew', padx=6)
        ttk.Button(frame, text="Browse...", command=self._pick_input).grid(row=0, column=2)

        ttk.Label(frame, text="Output .gcode:").grid(row=1, column=0, sticky='w', pady=2)
        ttk.Entry(frame, textvariable=self.output_path).grid(row=1, column=1, sticky='ew', padx=6)
        ttk.Button(frame, text="Browse...", command=self._pick_output).grid(row=1, column=2)

    def _build_mode_row(self):
        frame = ttk.LabelFrame(self, text="Mode", padding=8)
        frame.pack(fill='x', pady=(0, 8))
        ttk.Radiobutton(frame, text="Wall interrupt (pause the wall, grow the hair in place, resume)",
                        variable=self.mode, value='wall-interrupt').pack(anchor='w')
        ttk.Radiobutton(frame, text="Hair plugs (print each wall normally first, then apply all its hairs after)",
                        variable=self.mode, value='hair-plugs').pack(anchor='w')

    def _build_fuzzy_tools_section(self):
        frame = ttk.LabelFrame(self, text="Fuzzy Tools (up to 4 colors)", padding=8)
        frame.pack(fill='x', pady=(0, 8))

        note = ("Paint fuzzy regions to a tool number in your slicer, then set that "
                "number here. Leave all 4 blank to fuzz the whole model. Remap sends "
                "that tool's T-commands to a different real toolhead -- e.g. paint to "
                "an unused virtual T4, remap to a real T1.")
        ttk.Label(frame, text=note, wraplength=580, foreground="#555").grid(
            row=0, column=0, columnspan=4, sticky='w', pady=(0, 6))

        ttk.Label(frame, text="Fuzzy tool #").grid(row=1, column=0, sticky='w', padx=(0, 4))
        ttk.Label(frame, text="Remap to #").grid(row=1, column=2, sticky='w', padx=(12, 4))

        for i in range(FUZZY_TOOL_SLOTS):
            tool_var, remap_var = self.fuzzy_tool_vars[i]
            r = i + 2
            ttk.Label(frame, text=f"Slot {i + 1}:").grid(row=r, column=0, sticky='w', pady=2)
            tool_entry = ttk.Entry(frame, textvariable=tool_var, width=6)
            tool_entry.grid(row=r, column=1, sticky='w', padx=(4, 12))
            ttk.Label(frame, text="->").grid(row=r, column=2, sticky='e')
            remap_entry = ttk.Entry(frame, textvariable=remap_var, width=6)
            remap_entry.grid(row=r, column=3, sticky='w', padx=(4, 0))
            self._add_tooltip(tool_entry, f"Fuzzy-tool slot {i + 1}: only add loops while this "
                                           "tool (T0, T1, ...) is active. Leave blank to skip this slot.")
            self._add_tooltip(remap_entry, f"Rewrite every T-command that activates slot {i + 1}'s "
                                            "fuzzy tool to this real tool number. Leave blank to not "
                                            "remap (requires the tool # to its left to be set).")

    def _build_param_grid(self):
        frame = ttk.LabelFrame(self, text="Parameters", padding=8)
        frame.pack(fill='x', pady=(0, 8))

        cols = 2
        for i, (attr, label, kind, default, tip) in enumerate(FIELDS):
            row, col = divmod(i, cols)
            cell = ttk.Frame(frame)
            cell.grid(row=row, column=col, sticky='ew', padx=6, pady=3)
            frame.columnconfigure(col, weight=1)

            ttk.Label(cell, text=label, width=26).pack(side='left')
            var = tk.StringVar(value='' if default is None else str(default))
            entry = ttk.Entry(cell, textvariable=var, width=10)
            entry.pack(side='left', padx=(4, 0))
            self._add_tooltip(entry, tip)
            self.field_vars[attr] = (var, kind)

    def _build_flag_row(self):
        frame = ttk.LabelFrame(self, text="Options", padding=8)
        frame.pack(fill='x', pady=(0, 8))

        c1 = ttk.Checkbutton(frame, text="Random phase (vary where the first loop of each contour starts)",
                              variable=self.random_phase)
        c1.pack(anchor='w')

        c2 = ttk.Checkbutton(frame, text="Avoid collisions with other toolpaths (slower, buffers a full layer)",
                              variable=self.avoid_collisions)
        c2.pack(anchor='w')

    def _build_run_row(self):
        frame = ttk.Frame(self)
        frame.pack(fill='x', pady=(0, 8))
        self.run_button = ttk.Button(frame, text="Run", command=self._on_run)
        self.run_button.pack(side='left')
        self.progress = ttk.Progressbar(frame, mode='indeterminate', length=200)
        self.progress.pack(side='left', padx=10)

    def _build_log(self):
        frame = ttk.LabelFrame(self, text="Log", padding=8)
        frame.pack(fill='both', expand=True)
        self.log = tk.Text(frame, height=8, wrap='word', state='disabled')
        self.log.pack(fill='both', expand=True)

    # -- small helpers -------------------------------------------------

    def _add_tooltip(self, widget, text):
        tip = {'win': None}

        def show(_event):
            if tip['win'] is not None:
                return
            x = widget.winfo_rootx() + 10
            y = widget.winfo_rooty() + widget.winfo_height() + 4
            win = tk.Toplevel(widget)
            win.wm_overrideredirect(True)
            win.wm_geometry(f"+{x}+{y}")
            ttk.Label(win, text=text, background="#ffffe0", relief='solid',
                      borderwidth=1, padding=4, wraplength=280).pack()
            tip['win'] = win

        def hide(_event):
            if tip['win'] is not None:
                tip['win'].destroy()
                tip['win'] = None

        widget.bind('<Enter>', show)
        widget.bind('<Leave>', hide)

    # -- settings persistence -------------------------------------------

    def _load_settings(self):
        try:
            with open(SETTINGS_PATH, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError, OSError, ValueError):
            return

        for attr, (var, kind) in self.field_vars.items():
            if attr in data:
                var.set(str(data[attr]))
        if 'random_phase' in data:
            self.random_phase.set(bool(data['random_phase']))
        if 'avoid_collisions' in data:
            self.avoid_collisions.set(bool(data['avoid_collisions']))
        if 'mode' in data and data['mode'] in ('wall-interrupt', 'hair-plugs'):
            self.mode.set(data['mode'])
        if 'output_path' in data:
            self.output_path.set(data['output_path'])
        for i in range(FUZZY_TOOL_SLOTS):
            tool_var, remap_var = self.fuzzy_tool_vars[i]
            if f'fuzzy_tool_{i + 1}' in data:
                tool_var.set(str(data[f'fuzzy_tool_{i + 1}']))
            if f'remap_tool_{i + 1}_to' in data:
                remap_var.set(str(data[f'remap_tool_{i + 1}_to']))

    def _save_settings(self):
        data = {attr: var.get() for attr, (var, kind) in self.field_vars.items()}
        data['random_phase'] = self.random_phase.get()
        data['avoid_collisions'] = self.avoid_collisions.get()
        data['mode'] = self.mode.get()
        data['output_path'] = self.output_path.get()
        for i in range(FUZZY_TOOL_SLOTS):
            tool_var, remap_var = self.fuzzy_tool_vars[i]
            data[f'fuzzy_tool_{i + 1}'] = tool_var.get()
            data[f'remap_tool_{i + 1}_to'] = remap_var.get()
        try:
            with open(SETTINGS_PATH, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except OSError:
            pass  # best-effort -- a failed settings save shouldn't block a run

    def _log_line(self, text):
        self.log.configure(state='normal')
        self.log.insert('end', text + '\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def _pick_input(self):
        path = filedialog.askopenfilename(
            title="Select a G-code file",
            filetypes=[("G-code / sliced plate", "*.gcode *.gco *.g *.3mf"), ("All files", "*.*")])
        if not path:
            return
        self.input_path.set(path)
        if not self.output_path.get():
            base, ext = gcode3mf.split_ext(path)
            self.output_path.set(f"{base}_fuzzed{ext or '.gcode'}")

    def _pick_output(self):
        initial = self.output_path.get() or self.input_path.get()
        initdir = os.path.dirname(initial) if initial else None
        initfile = os.path.basename(initial) if initial else "output_fuzzed.gcode"
        # follow whatever the input is, so a .gcode.3mf saves back as one
        ext = gcode3mf.split_ext(self.input_path.get())[1] or '.gcode'
        path = filedialog.asksaveasfilename(
            title="Save fuzzed G-code as",
            defaultextension=ext,
            initialdir=initdir, initialfile=initfile,
            filetypes=[("Bambu sliced plate", "*.gcode.3mf *.3mf"),
                       ("G-code files", "*.gcode *.gco *.g"),
                       ("All files", "*.*")])
        if path:
            self.output_path.set(path)

    # -- validation / run -----------------------------------------------

    def _parse_field(self, attr, label, kind, raw):
        raw = raw.strip()
        if kind == 'float':
            if raw == '':
                raise ValueError(f"{label} can't be blank.")
            return float(raw)
        if kind == 'int':
            if raw == '':
                raise ValueError(f"{label} can't be blank.")
            return int(raw)
        if kind == 'optional_float':
            return None if raw == '' else float(raw)
        if kind == 'optional_int':
            return None if raw == '' else int(raw)
        raise ValueError(f"unknown field kind {kind}")

    def _build_args(self):
        if not self.input_path.get():
            raise ValueError("Choose an input .gcode file.")
        if not os.path.isfile(self.input_path.get()):
            raise ValueError("Input file doesn't exist.")
        if not self.output_path.get():
            raise ValueError("Choose an output file name.")

        args = Args()
        args.input = self.input_path.get()
        args.output = self.output_path.get()
        args.random_phase = self.random_phase.get()
        args.avoid_collisions = self.avoid_collisions.get()
        args.mode = self.mode.get()

        for attr, label, kind, default, _tip in FIELDS:
            var, kind = self.field_vars[attr]
            value = self._parse_field(attr, label, kind, var.get())
            setattr(args, attr, value)

        # match the CLI's "no limit" convention
        if args.max_z is None:
            args.max_z = float('inf')

        for i in range(FUZZY_TOOL_SLOTS):
            tool_var, remap_var = self.fuzzy_tool_vars[i]
            tool_val = self._parse_field(f'fuzzy_tool_{i + 1}', f'Slot {i + 1} fuzzy tool #',
                                          'optional_int', tool_var.get())
            remap_val = self._parse_field(f'remap_tool_{i + 1}_to', f'Slot {i + 1} remap to #',
                                           'optional_int', remap_var.get())
            if remap_val is not None and tool_val is None:
                raise ValueError(f'Fuzzy Tools slot {i + 1}: "Remap to #" needs a "Fuzzy '
                                  f'tool #" set on the same row -- otherwise there\'s '
                                  f'nothing to remap.')
            setattr(args, f'fuzzy_tool_{i + 1}', tool_val)
            setattr(args, f'remap_tool_{i + 1}_to', remap_val)

        return args

    def _on_run(self):
        try:
            args = self._build_args()
        except ValueError as e:
            messagebox.showerror("Check your inputs", str(e))
            return

        if gcode3mf.is_3mf(args.output) and not gcode3mf.is_3mf(args.input):
            messagebox.showerror(
                "Can't save as .gcode.3mf",
                "A sliced plate file also contains plate metadata, thumbnails and "
                "the model, which can only be copied from an existing .gcode.3mf.\n\n"
                "Either pick a .gcode.3mf as the input, or change the output to "
                ".gcode.")
            return

        self._save_settings()

        self.run_button.configure(state='disabled')
        self.progress.start(12)
        self._log_line(f"Running on {os.path.basename(args.input)} "
                        f"({'collision-aware' if args.avoid_collisions else 'fast'} mode)...")

        thread = threading.Thread(target=self._run_worker, args=(args,), daemon=True)
        thread.start()

    def _run_worker(self, args):
        try:
            before_loops = hairy_walls.fuzzify_contour.stats_loops
            before_skipped = hairy_walls.fuzzify_contour.stats_skipped
            before_contours = hairy_walls.fuzzify_contour.stats_contours

            lines = gcode3mf.read_lines(args.input)

            if args.avoid_collisions:
                result = hairy_walls.process_collision_aware(lines, args)
            else:
                result = hairy_walls.process(lines, args)

            gcode3mf.write_lines(args.output, result, args.input)

            loops = hairy_walls.fuzzify_contour.stats_loops - before_loops
            skipped = hairy_walls.fuzzify_contour.stats_skipped - before_skipped
            contours = hairy_walls.fuzzify_contour.stats_contours - before_contours

            msg = f"Done: {loops} loops added across {contours} outer-wall contours"
            if args.avoid_collisions:
                msg += f", {skipped} dropped to avoid collisions"
            msg += f"\nSaved to {args.output}"
            self.master.after(0, self._on_success, msg)
        except Exception:
            err = traceback.format_exc()
            self.master.after(0, self._on_failure, err)

    def _on_success(self, msg):
        self.progress.stop()
        self.run_button.configure(state='normal')
        self._log_line(msg)
        messagebox.showinfo("Done", msg)

    def _on_failure(self, err):
        self.progress.stop()
        self.run_button.configure(state='normal')
        self._log_line("Error:\n" + err)
        messagebox.showerror("Something went wrong", err.strip().splitlines()[-1])


def main():
    root = tk.Tk()
    root.title("Hairy Walls")
    root.geometry("700x760")

    # scrollable container: the window now has more sections than fit
    # comfortably (Fuzzy Tools, Mode, Parameters, ...), so wrap everything
    # in a canvas + scrollbar rather than relying on the window being
    # resized tall enough
    container = ttk.Frame(root)
    container.pack(fill='both', expand=True)

    canvas = tk.Canvas(container, highlightthickness=0)
    scrollbar = ttk.Scrollbar(container, orient='vertical', command=canvas.yview)
    canvas.configure(yscrollcommand=scrollbar.set)
    scrollbar.pack(side='right', fill='y')
    canvas.pack(side='left', fill='both', expand=True)

    inner = ttk.Frame(canvas)
    inner_id = canvas.create_window((0, 0), window=inner, anchor='nw')

    def _sync_scrollregion(_event=None):
        canvas.configure(scrollregion=canvas.bbox('all'))

    def _sync_inner_width(event):
        canvas.itemconfig(inner_id, width=event.width)

    inner.bind('<Configure>', _sync_scrollregion)
    canvas.bind('<Configure>', _sync_inner_width)

    def _on_mousewheel(event):
        if event.delta:
            canvas.yview_scroll(-1 if event.delta > 0 else 1, 'units')
    def _on_mousewheel_linux(event):
        canvas.yview_scroll(-1 if event.num == 4 else 1, 'units')

    canvas.bind_all('<MouseWheel>', _on_mousewheel)          # Windows / macOS
    canvas.bind_all('<Button-4>', _on_mousewheel_linux)       # Linux scroll up
    canvas.bind_all('<Button-5>', _on_mousewheel_linux)       # Linux scroll down

    HairyWallsGUI(inner)
    root.mainloop()


if __name__ == '__main__':
    main()
