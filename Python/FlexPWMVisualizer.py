#!/usr/bin/env python3

# Copyright (c) 2026 Mitchell C. Nelson
# Portions of logic and visualization architecture co-developed with Gemini AI.

#!/usr/bin/env python3

# Copyright (c) 2026 Mitchell C. Nelson
# Portions of logic and visualization architecture co-developed with Gemini AI.

import re
import sys
import numpy as np
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.ticker import AutoMinorLocator
import os
import signal

class PWMSubmodule:
    def __init__(self, name="UNKNOWN"):
        self.name = name
        self.period = 0
        self.presc = 0
        self.offset = 0.0  # Added as requested
        self.onA = 0; self.offA = 0; self.pinA = "?"; self.invA = 0
        self.onB = 0; self.offB = 0; self.pinB = "?"; self.invB = 0

    def parse_fragment(self, suffix, val_str):
        # Extract key-value pairs like 'on 37', 'pin 4', or 'offset 0.9'
        # Updated regex to handle float offsets
        matches = re.findall(r'([a-zA-Z_]+)\s+([-+]?\d*\.?\d+)', val_str)
        data = {k.lower(): float(v) if '.' in v else int(v) for k, v in matches}
        
        if suffix == 'period':
            period_match = re.search(r'^\s*(\d+)', val_str)
            if period_match: self.period = int(period_match.group(1))
            self.presc = data.get('presc', 0)
            self.offset = data.get('offset', self.offset) # Update offset if provided
        elif suffix == 'a':
            self.onA = data.get('on', 0); self.offA = data.get('off', 0)
            self.pinA = str(data.get('pin', "?"))
            self.offset = data.get('offset', self.offset) # Update offset if provided
            self.invA = 1 if 'inverting' in val_str.lower() and 'noninverting' not in val_str.lower() else 0
        elif suffix == 'b':
            self.onB = data.get('on', 0); self.offB = data.get('off', 0)
            self.pinB = str(data.get('pin', "?"))
            self.offset = data.get('offset', self.offset) # Update offset if provided
            self.invB = 1 if 'inverting' in val_str.lower() and 'noninverting' not in val_str.lower() else 0

    def get_waveform(self, t_array, tick_ns, channel='A'):
        on = self.onA if channel == 'A' else self.onB
        off = self.offA if channel == 'A' else self.offB
        inv = self.invA if channel == 'A' else self.invB
        
        if self.period <= 0: return np.zeros_like(t_array)
        if on == off: return np.ones_like(t_array) if inv else np.zeros_like(t_array)

        sm_tick = tick_ns * (2**self.presc)
        
        # Apply the hardware offset here
        # We subtract offset from t_array so the signal starts 'later'
        t_shifted = t_array - self.offset
        
        # Any time before the offset is forced to the 'off' (or inverted 'on') state
        # to prevent wrap-around artifacts from the modulo operator
        cnt = (t_shifted / sm_tick) % self.period
        
        pwm = np.where((cnt >= on) & (cnt < off), 1, 0) if on < off else np.where((cnt >= on) | (cnt < off), 1, 0)
        
        # Force 0 before the offset starts
        pwm = np.where(t_shifted >= 0, pwm, 0)
        
        return 1 - pwm if inv else pwm

class FlexPWMVisualizer:
    def __init__(self, f_bus_mhz=150):
        self.f_bus = f_bus_mhz
        self.submodules = {}

    def parse_line(self, line):
        # Handles various styles including hardware: flexpwm: clk A pin 4 offset 0.9...
        match = re.search(
            r'(?:#\s*)?flexpwm[_\s:]+(\w+)[_\s]+(period|A|B)(?:\s*=\s*["\']|\s+|:\s+)(.*?)(?:["\']|$)', 
            line, 
            re.IGNORECASE
        )
        
        if match:
            name = match.group(1).upper()
            suffix = match.group(2).lower()
            content = match.group(3)
            
            if name not in self.submodules:
                self.submodules[name] = PWMSubmodule(name)
            
            self.submodules[name].parse_fragment(suffix, content)
            return True
        return False
    
    def get_figure(self):
        fig = Figure(figsize=(12, 8), dpi=100)
        ax = fig.add_subplot(111)
        if not self.submodules: return fig

        tick_ns = 1000 / self.f_bus
        # Account for offsets in durations to ensure the plot window is wide enough
        durations = [(sm.period * (2**sm.presc) * tick_ns) + sm.offset for sm in self.submodules.values() if sm.period > 0]
        max_ns = max(durations + [1000])
        
        t = np.linspace(0, max_ns, 15000)
        y_offset = 0
        for name in sorted(self.submodules.keys(), reverse=True):
            sm = self.submodules[name]
            for ch in ['B', 'A']:
                on = sm.onA if ch == 'A' else sm.onB
                off = sm.offA if ch == 'A' else sm.offB
                pin = sm.pinA if ch == 'A' else sm.pinB
                
                if ch == 'B' and on == off: continue
                
                pwm = sm.get_waveform(t, tick_ns, channel=ch)
                ax.step(t/1000, pwm + y_offset, where='post', lw=1.5)
                
                pin_display = "No Pin" if pin == "255" else f"Pin {pin}"
                label = f"{name}_{ch}\n({pin_display})\nOffset: {sm.offset}ns" if sm.offset > 0 else f"{name}_{ch}\n({pin_display})"
                ax.text(-max_ns*0.02/1000, y_offset + 0.5, label, ha='right', va='center', weight='bold', fontsize=8)
                y_offset += 1.5

        ax.xaxis.set_minor_locator(AutoMinorLocator(5))
        ax.grid(visible=True, which='both', axis='x', color='#666666', alpha=0.2)
        ax.set_title("TCD1304 Timing Sequence (Hardware Calibrated Offsets)")
        ax.set_xlabel("Time (µs)")
        ax.set_yticks([]) 
        ax.set_xlim(-max_ns*0.25/1000, max_ns/1000)
        fig.tight_layout()
        return fig

    def popup(self, top):
        fig = self.get_figure()
        canvas = FigureCanvasTkAgg(fig, master=top)
        canvas.draw()
        
        toolbar = NavigationToolbar2Tk(canvas, top)
        toolbar.update()
        toolbar.pack(side=tk.BOTTOM, fill=tk.X)

        canvas_widget = canvas.get_tk_widget()
        canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        top.lift()

def SignalHandler(sig, frame): os._exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, SignalHandler)
    root = tk.Tk()
    root.title("FlexPWM Standalone Visualizer")
    
    viz = FlexPWMVisualizer()
    
    # Load data from file if provided as a command line argument
    if len(sys.argv) > 1:
        if os.path.exists(sys.argv[1]):
            with open(sys.argv[1], 'r') as f:
                for line in f:
                    viz.parse_line(line)
        else:
            print(f"File not found: {sys.argv[1]}")

    if viz.submodules:
        icg = viz.submodules['ICG']
        print( 'icg', icg.__dict__.items() )
        cnvst = viz.submodules['CNVST']
        print( 'cnvst', cnvst.__dict__.items() )

        if cnvst.offset == 0:
            cnvst.offset = (icg.offA * (1000. / viz.f_bus) * (2**icg.presc)) + 900

        canvas = FigureCanvasTkAgg(viz.get_figure(), master=root)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)
        
        toolbar = NavigationToolbar2Tk(canvas, root)
        toolbar.update()
    else:
        tk.Label(root, text="No FlexPWM data found in file.", padx=20, pady=20).pack()

    root.mainloop()

    
