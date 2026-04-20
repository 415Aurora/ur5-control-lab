import customtkinter as ctk
import numpy as np

class Matrix4x4View(ctk.CTkFrame):
    """
    A 4x4 Grid of Labels to display a transformation matrix neatly.
    """
    def __init__(self, master, title="Transformation Matrix", **kwargs):
        super().__init__(master, **kwargs)
        
        # Title
        self.lbl_title = ctk.CTkLabel(self, text=title, font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_title.grid(row=0, column=0, columnspan=4, pady=(10, 5), sticky="ew")
        
        # Grid Container
        self.grid_container = ctk.CTkFrame(self, fg_color="transparent")
        self.grid_container.grid(row=1, column=0, columnspan=4, padx=10, pady=10)
        
        self.cells = []
        for r in range(4):
            row_cells = []
            for c in range(4):
                # Styled cell
                cell = ctk.CTkLabel(
                    self.grid_container, 
                    text="0.00", 
                    width=75, 
                    height=40,
                    fg_color=("gray85", "gray25"), 
                    corner_radius=4,
                    font=ctk.CTkFont(family="Monospace", size=14, weight="bold")
                )
                cell.grid(row=r, column=c, padx=3, pady=3, sticky="nsew") # Less padding for tighter grid
                row_cells.append(cell)
            self.cells.append(row_cells)

    def update_matrix(self, matrix):
        if matrix is None: return
        for r in range(4):
            for c in range(4):
                val = matrix[r, c]
                color = "gray25"
                fg = "white"
                if c == 3: 
                    color = "#444444" 
                    fg = "#50FA7B" 
                self.cells[r][c].configure(text=f"{val:.3f}", fg_color=color, text_color=fg)


class DHTableView(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        headers = ["Joint", "Theta", "d", "a", "Alpha"]
        for i, h in enumerate(headers):
            lbl = ctk.CTkLabel(self, text=h, font=ctk.CTkFont(size=13, weight="bold"))
            lbl.grid(row=0, column=i, padx=5, pady=8, sticky="ew")
            self.grid_columnconfigure(i, weight=1)
        self.rows = []
        for r in range(6):
            row_widgets = []
            for c in range(5):
                lbl = ctk.CTkLabel(self, text="--", font=ctk.CTkFont(family="Monospace", size=12))
                lbl.grid(row=r+1, column=c, padx=5, pady=5)
                row_widgets.append(lbl)
            self.rows.append(row_widgets)

    def update_table(self, dh_data):
        for i, row in enumerate(dh_data):
            if i >= 6: break
            vals = [
                row.get('Joint', i+1),
                f"{float(row.get('theta (rad)', 0)):.3f}",
                f"{float(row.get('d (m)', 0)):.4f}",
                f"{float(row.get('a (m)', 0)):.4f}",
                f"{float(row.get('alpha (rad)', 0)):.3f}"
            ]
            for c, val in enumerate(vals):
                self.rows[i][c].configure(text=str(val))


class JointInput(ctk.CTkFrame):
    """
    Horizontal Stepper widget for a single Joint.
    Layout: [ Label ]  [ - ] [ Value ] [ + ]
    """
    def __init__(self, master, joint_idx, joint_name="Joint", initial_val=0, command=None, **kwargs):
        super().__init__(master, **kwargs)
        self.val = initial_val
        self.idx = joint_idx
        self.command = command
        
        # Grid Layout
        self.grid_columnconfigure(0, weight=1) # Label stretches
        self.grid_columnconfigure(4, weight=0) # Spacer
        
        # Title
        ctk.CTkLabel(self, text=f"J{joint_idx+1} ({joint_name})", font=ctk.CTkFont(size=14, weight="bold")).grid(row=0, column=0, padx=10, sticky="w")
        
        # Controls
        ctk.CTkButton(self, text="-", width=40, height=30, command=lambda: self._adj(-1)).grid(row=0, column=1, padx=5)
        
        self.entry = ctk.CTkEntry(self, width=60, justify="center", font=ctk.CTkFont(size=14))
        self.entry.insert(0, str(initial_val))
        self.entry.grid(row=0, column=2, padx=5)
        
        ctk.CTkButton(self, text="+", width=40, height=30, command=lambda: self._adj(1)).grid(row=0, column=3, padx=5)

    def _adj(self, delta):
        try:
            curr = float(self.entry.get())
            new_val = int(curr + delta)
            self.entry.delete(0, "end")
            self.entry.insert(0, str(new_val))
            if self.command: self.command(self.idx, new_val)
        except ValueError:
            pass

    def get_value(self):
        try:
            return float(self.entry.get())
        except ValueError:
            return 0.0

