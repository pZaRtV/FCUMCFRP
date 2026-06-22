"""
FCU Flight Log Analysis — GUI Tool
Graphical wrapper around FlightlogAnalysis.py analysis functions.
Provides tabbed plots, checkbox-controlled analysis, and results sidebar.

Dependencies: tkinter (built-in), matplotlib, numpy, pandas (already required)
Launch:  python FlightlogGUI.py

Author: Patrick Andrasena T.
"""

import os
import sys
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np

# Import analysis functions from sibling module
import FlightlogAnalysis as fla

# ── Theme colours ────────────────────────────────────────────────────────────
BG           = "#1e1e2e"
BG_PANEL     = "#282840"
BG_CARD      = "#313150"
FG           = "#cdd6f4"
FG_DIM       = "#6c7086"
ACCENT_GREEN = "#a6e3a1"
ACCENT_BLUE  = "#89b4fa"
ACCENT_RED   = "#f38ba8"
ACCENT_PEACH = "#fab387"
ACCENT_MAUVE = "#cba6f7"
BORDER       = "#45475a"

# ── Analysis registry (checkbox label → function to call) ────────────────────
ANALYSIS_ITEMS = [
    ("Data Integrity",      "integrity"),
    ("Basic Statistics",     "statistics"),
    ("Stability Metrics",   "stability"),
    ("Cross-Correlation",   "correlation"),
    ("Madgwick β Analysis", "beta_analysis"),
    ("β Parameter Sweep",   "beta_sweep"),
]

PLOT_ITEMS = [
    ("Main Attitude",       "attitude"),
    ("Error Analysis",      "error"),
    ("Per-IMU Streams",     "per_imu"),
    ("Sensor Comparison",   "sensor_diff"),
    ("Data Integrity",      "integrity_plot"),
    ("Correlation Heatmap", "corr_heatmap"),
    ("Frequency Analysis",  "frequency"),
    ("β Sweep Curves",      "beta_sweep_plot"),
]


class FlightLogGUI(tk.Tk):
    """Main application window."""

    def __init__(self):
        super().__init__()
        self.title("FCU Flight Log Analysis Tool")
        self.geometry("1400x900")
        self.configure(bg=BG)
        self.minsize(1100, 700)

        # State (UDPClient session folder: imu_data.csv required, event log optional)
        self.log_folder = None
        self.csv_path = None
        self.event_log_path = None
        self.df = None
        self.metrics = None
        self.integrity = None
        self.correlation_metrics = None
        self.beta_sweep_results = None
        self.madgwick_analysis = None
        self.analysis_vars = {}
        self.plot_vars = {}

        self._build_ui()

    # ── UI Construction ──────────────────────────────────────────────────────

    def _build_ui(self):
        # Top toolbar
        self._build_toolbar()

        # Main content: left panel + center plots + right sidebar
        content = tk.Frame(self, bg=BG)
        content.pack(fill=tk.BOTH, expand=True, padx=4, pady=(0, 4))
        content.columnconfigure(1, weight=1)
        content.rowconfigure(0, weight=1)

        self._build_left_panel(content)
        self._build_center_area(content)
        self._build_right_sidebar(content)
        self._build_status_bar()

    def _build_toolbar(self):
        toolbar = tk.Frame(self, bg=BG_PANEL, height=44)
        toolbar.pack(fill=tk.X, padx=4, pady=4)
        toolbar.pack_propagate(False)

        btn_open = tk.Button(toolbar, text="📂 Open Log Folder", command=self._open_folder,
                             bg=ACCENT_BLUE, fg="#1e1e2e", font=("Segoe UI", 10, "bold"),
                             relief=tk.FLAT, padx=12, pady=4, cursor="hand2")
        btn_open.pack(side=tk.LEFT, padx=8, pady=6)

        self.lbl_path = tk.Label(toolbar, text="No session folder loaded", bg=BG_PANEL, fg=FG_DIM,
                                 font=("Consolas", 9), anchor="w")
        self.lbl_path.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

        btn_export = tk.Button(toolbar, text="💾 Export Report", command=self._export_report,
                               bg=ACCENT_GREEN, fg="#1e1e2e", font=("Segoe UI", 10, "bold"),
                               relief=tk.FLAT, padx=12, pady=4, cursor="hand2")
        btn_export.pack(side=tk.RIGHT, padx=8, pady=6)

        btn_run = tk.Button(toolbar, text="▶ Run Analysis", command=self._run_analysis,
                            bg=ACCENT_PEACH, fg="#1e1e2e", font=("Segoe UI", 10, "bold"),
                            relief=tk.FLAT, padx=12, pady=4, cursor="hand2")
        btn_run.pack(side=tk.RIGHT, padx=4, pady=6)

    def _build_left_panel(self, parent):
        panel = tk.Frame(parent, bg=BG_PANEL, width=220)
        panel.grid(row=0, column=0, sticky="ns", padx=(0, 4))
        panel.grid_propagate(False)

        # ── Analysis checkboxes ──
        lbl_a = tk.Label(panel, text="ANALYSIS", bg=BG_PANEL, fg=ACCENT_BLUE,
                         font=("Segoe UI", 9, "bold"))
        lbl_a.pack(anchor="w", padx=10, pady=(10, 4))

        for label, key in ANALYSIS_ITEMS:
            var = tk.BooleanVar(value=True)
            self.analysis_vars[key] = var
            cb = tk.Checkbutton(panel, text=label, variable=var,
                                bg=BG_PANEL, fg=FG, selectcolor=BG_CARD,
                                activebackground=BG_PANEL, activeforeground=FG,
                                font=("Segoe UI", 9), anchor="w")
            cb.pack(fill=tk.X, padx=14, pady=1)

        ttk.Separator(panel, orient="horizontal").pack(fill=tk.X, padx=10, pady=8)

        # ── Plot checkboxes ──
        lbl_p = tk.Label(panel, text="PLOTS", bg=BG_PANEL, fg=ACCENT_GREEN,
                         font=("Segoe UI", 9, "bold"))
        lbl_p.pack(anchor="w", padx=10, pady=(0, 4))

        for label, key in PLOT_ITEMS:
            var = tk.BooleanVar(value=True)
            self.plot_vars[key] = var
            cb = tk.Checkbutton(panel, text=label, variable=var,
                                bg=BG_PANEL, fg=FG, selectcolor=BG_CARD,
                                activebackground=BG_PANEL, activeforeground=FG,
                                font=("Segoe UI", 9), anchor="w")
            cb.pack(fill=tk.X, padx=14, pady=1)

        ttk.Separator(panel, orient="horizontal").pack(fill=tk.X, padx=10, pady=8)

        # Select / deselect all
        btn_frame = tk.Frame(panel, bg=BG_PANEL)
        btn_frame.pack(fill=tk.X, padx=10)
        tk.Button(btn_frame, text="Select All", command=self._select_all,
                  bg=BG_CARD, fg=FG, relief=tk.FLAT, font=("Segoe UI", 8),
                  cursor="hand2").pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(btn_frame, text="Deselect All", command=self._deselect_all,
                  bg=BG_CARD, fg=FG, relief=tk.FLAT, font=("Segoe UI", 8),
                  cursor="hand2").pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

    def _build_center_area(self, parent):
        """Scrollable canvas that holds multiple persistent plot frames."""
        container = tk.Frame(parent, bg=BG)
        container.grid(row=0, column=1, sticky="nsew")
        container.rowconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)

        self.plot_canvas = tk.Canvas(container, bg=BG, highlightthickness=0)
        scrollbar = tk.Scrollbar(container, orient=tk.VERTICAL, command=self.plot_canvas.yview)
        self.plot_canvas.configure(yscrollcommand=scrollbar.set)

        scrollbar.grid(row=0, column=1, sticky="ns")
        self.plot_canvas.grid(row=0, column=0, sticky="nsew")

        self.plot_frame = tk.Frame(self.plot_canvas, bg=BG)
        self.plot_window_id = self.plot_canvas.create_window((0, 0), window=self.plot_frame,
                                                              anchor="nw")

        self.plot_frame.bind("<Configure>", self._on_plot_frame_configure)
        self.plot_canvas.bind("<Configure>", self._on_canvas_configure)
        # Mouse wheel scroll
        self.plot_canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        # Placeholder text
        self.placeholder = tk.Label(
            self.plot_frame,
            text="Select a UDPClient session folder\n(imu_data.csv required, event_log optional)\nthen run analysis to display plots here",
            bg=BG, fg=FG_DIM, font=("Segoe UI", 14), justify="center",
        )
        self.placeholder.pack(expand=True, fill=tk.BOTH, pady=200)

    def _build_right_sidebar(self, parent):
        sidebar = tk.Frame(parent, bg=BG_PANEL, width=280)
        sidebar.grid(row=0, column=2, sticky="ns", padx=(4, 0))
        sidebar.grid_propagate(False)

        lbl = tk.Label(sidebar, text="RESULTS", bg=BG_PANEL, fg=ACCENT_MAUVE,
                       font=("Segoe UI", 10, "bold"))
        lbl.pack(anchor="w", padx=10, pady=(10, 6))

        # Scrollable results text
        self.results_text = tk.Text(sidebar, bg=BG_CARD, fg=FG, font=("Consolas", 9),
                                    wrap=tk.WORD, relief=tk.FLAT, padx=8, pady=8,
                                    insertbackground=FG, state=tk.DISABLED)
        results_scroll = tk.Scrollbar(sidebar, command=self.results_text.yview)
        self.results_text.configure(yscrollcommand=results_scroll.set)
        results_scroll.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 4), pady=(0, 4))
        self.results_text.pack(fill=tk.BOTH, expand=True, padx=(4, 0), pady=(0, 4))

    def _build_status_bar(self):
        self.status_var = tk.StringVar(value="Ready")
        bar = tk.Label(self, textvariable=self.status_var, bg=BG_PANEL, fg=FG_DIM,
                       font=("Consolas", 9), anchor="w", padx=10)
        bar.pack(fill=tk.X, side=tk.BOTTOM)

    # ── Scroll helpers ───────────────────────────────────────────────────────

    def _on_plot_frame_configure(self, event):
        self.plot_canvas.configure(scrollregion=self.plot_canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self.plot_canvas.itemconfig(self.plot_window_id, width=event.width)

    def _on_mousewheel(self, event):
        self.plot_canvas.yview_scroll(-1 * (event.delta // 120), "units")

    # ── Actions ──────────────────────────────────────────────────────────────

    def _select_all(self):
        for v in list(self.analysis_vars.values()) + list(self.plot_vars.values()):
            v.set(True)

    def _deselect_all(self):
        for v in list(self.analysis_vars.values()) + list(self.plot_vars.values()):
            v.set(False)

    @staticmethod
    def _resolve_session_folder(folder):
        """
        Locate UDPClient session files in a folder.
        Returns (csv_path, event_log_path_or_none, error_message_or_none).
        """
        folder = os.path.abspath(folder)
        if not os.path.isdir(folder):
            return None, None, "Not a directory."

        csv_path = os.path.join(folder, fla.CSV_FILENAME)
        if not os.path.isfile(csv_path):
            csv_files = sorted(
                f for f in os.listdir(folder)
                if f.lower().endswith(".csv")
            )
            if len(csv_files) == 1:
                csv_path = os.path.join(folder, csv_files[0])
            elif len(csv_files) == 0:
                return None, None, (
                    f"No CSV found in folder.\nExpected {fla.CSV_FILENAME} "
                    f"(from UDPClient.py)."
                )
            else:
                return None, None, (
                    "Multiple CSV files in folder; expected a single session "
                    f"with {fla.CSV_FILENAME}:\n" + "\n".join(csv_files)
                )

        event_log_path = None
        for name in ("event_log.txt", "event_log.log"):
            candidate = os.path.join(folder, name)
            if os.path.isfile(candidate):
                event_log_path = candidate
                break
        if event_log_path is None:
            log_files = sorted(
                f for f in os.listdir(folder)
                if f.lower().endswith((".log", ".txt"))
                and not f.lower().endswith(".csv")
                and "event" in f.lower()
            )
            if len(log_files) == 1:
                event_log_path = os.path.join(folder, log_files[0])

        return csv_path, event_log_path, None

    def _open_folder(self):
        folder = filedialog.askdirectory(
            title="Select UDP session folder (imu_data.csv + optional event log)",
            initialdir=os.path.join(os.path.dirname(os.path.abspath(__file__)), "data_logs"),
        )
        if not folder:
            return

        csv_path, event_log_path, err = self._resolve_session_folder(folder)
        if err:
            messagebox.showerror("Invalid Session Folder", err)
            return

        self.log_folder = folder
        self.csv_path = csv_path
        self.event_log_path = event_log_path

        folder_name = os.path.basename(folder.rstrip(os.sep))
        csv_name = os.path.basename(csv_path)
        if event_log_path:
            status = f"{folder_name}  |  {csv_name} + {os.path.basename(event_log_path)}"
        else:
            status = f"{folder_name}  |  {csv_name} (no event log)"
        self.lbl_path.config(text=status, fg=FG)
        self._load_data()

    def _load_data(self):
        self.status_var.set("Loading session CSV...")
        self.update_idletasks()
        try:
            self.df = fla.load_csv_data(self.csv_path)
            if self.df is None or len(self.df) == 0:
                messagebox.showerror("Error", "No data found in the session CSV.")
                return
            msg = f"Loaded {len(self.df)} rows from {os.path.basename(self.csv_path)}"
            if self.event_log_path:
                msg += f" (+ {os.path.basename(self.event_log_path)})"
            self.status_var.set(msg)
        except Exception as e:
            messagebox.showerror("Load Error", str(e))
            self.status_var.set("Load failed")

    def _run_analysis(self):
        if self.df is None:
            messagebox.showwarning("No Data", "Open a UDP session folder first.")
            return
        # Run in thread to keep UI responsive
        self.status_var.set("Running analysis...")
        self.update_idletasks()
        thread = threading.Thread(target=self._analysis_worker, daemon=True)
        thread.start()

    def _analysis_worker(self):
        """Run selected analyses, then update UI on main thread."""
        try:
            results_lines = []
            df = self.df

            # ── Analysis computations ──
            if self.analysis_vars["integrity"].get():
                self.integrity = fla.validate_data_integrity(df)
                results_lines.append(self._format_integrity(self.integrity))

            if self.analysis_vars["stability"].get():
                self.metrics = fla.calculate_advanced_stability_metrics(df)
                results_lines.append(self._format_stability(self.metrics))

            if self.analysis_vars["correlation"].get():
                self.correlation_metrics = fla.calculate_cross_correlation_metrics(df)
                results_lines.append(self._format_correlation(self.correlation_metrics))

            if self.analysis_vars["beta_analysis"].get():
                if self.metrics is None:
                    self.metrics = fla.calculate_advanced_stability_metrics(df)
                self.madgwick_analysis = fla.analyze_madgwick_beta_correlation(df, self.metrics)
                results_lines.append(self._format_beta_analysis(self.madgwick_analysis))

            if self.analysis_vars["beta_sweep"].get():
                self.beta_sweep_results = fla.sweep_madgwick_beta(df)
                if self.beta_sweep_results:
                    results_lines.append(self._format_beta_sweep(self.beta_sweep_results))

            if self.analysis_vars["statistics"].get():
                results_lines.append(self._format_statistics(df))

            if self.event_log_path:
                results_lines.append(self._format_event_log_summary(self.event_log_path))

            results_text = "\n".join(results_lines)

            # ── Generate plots ──
            plot_figs = []

            if self.plot_vars["attitude"].get():
                fig = fla.create_analysis_plots(df)
                if fig:
                    plot_figs.append(("Main Attitude & Sensor Overview", fig))

            if self.plot_vars["error"].get():
                if self.metrics is None:
                    self.metrics = fla.calculate_advanced_stability_metrics(df)
                fig = fla.create_error_analysis_plot(df, self.metrics)
                if fig:
                    plot_figs.append(("Error Analysis", fig))

            if self.plot_vars["per_imu"].get():
                fig = fla.create_per_imu_plots(df)
                if fig:
                    plot_figs.append(("Per-IMU Raw Streams", fig))

            if self.plot_vars["sensor_diff"].get():
                fig = fla.create_sensor_difference_plots(df)
                if fig:
                    plot_figs.append(("Sensor Comparison (diff_*)", fig))

            if self.plot_vars["integrity_plot"].get():
                if self.integrity is None:
                    self.integrity = fla.validate_data_integrity(df)
                fig = fla.create_data_integrity_plot(df, self.integrity)
                if fig:
                    plot_figs.append(("Data Integrity", fig))

            if self.plot_vars["corr_heatmap"].get():
                fig = fla.create_correlation_heatmap(df)
                if fig:
                    plot_figs.append(("Correlation Heatmap", fig))

            if self.plot_vars["frequency"].get():
                if self.metrics is None:
                    self.metrics = fla.calculate_advanced_stability_metrics(df)
                fig = fla.create_frequency_analysis_plot(df, self.metrics)
                if fig:
                    plot_figs.append(("Frequency Analysis", fig))

            if self.plot_vars["beta_sweep_plot"].get() and self.beta_sweep_results:
                fig = fla.plot_beta_sweep_results(self.beta_sweep_results)
                if fig:
                    plot_figs.append(("β Parameter Sweep", fig))

            # Update UI on main thread
            self.after(0, lambda: self._display_results(results_text, plot_figs))

        except Exception as e:
            self.after(0, lambda: self._on_analysis_error(str(e)))

    # ── Display helpers ──────────────────────────────────────────────────────

    def _display_results(self, results_text, plot_figs):
        """Update results sidebar and plot canvas (called on main thread)."""
        # Clear placeholder
        self.placeholder.destroy()

        # Update results sidebar
        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete("1.0", tk.END)
        self.results_text.insert(tk.END, results_text)
        self.results_text.config(state=tk.DISABLED)

        # Clear previous plots
        for widget in self.plot_frame.winfo_children():
            widget.destroy()

        # Embed each figure as a persistent plot with its own toolbar
        for title, fig in plot_figs:
            self._embed_plot(title, fig)

        self.status_var.set(f"Analysis complete — {len(plot_figs)} plots generated")

    def _embed_plot(self, title, fig):
        """Add a titled matplotlib figure to the scrollable plot area."""
        wrapper = tk.Frame(self.plot_frame, bg=BG_CARD, bd=1, relief=tk.FLAT,
                           highlightbackground=BORDER, highlightthickness=1)
        wrapper.pack(fill=tk.X, padx=6, pady=6)

        lbl = tk.Label(wrapper, text=title, bg=BG_CARD, fg=ACCENT_BLUE,
                       font=("Segoe UI", 11, "bold"), anchor="w")
        lbl.pack(fill=tk.X, padx=10, pady=(8, 0))

        # Resize figure to fit canvas width
        fig.set_size_inches(12, fig.get_size_inches()[1])
        fig.set_facecolor(BG_CARD)

        canvas = FigureCanvasTkAgg(fig, master=wrapper)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.X, padx=4, pady=4)

        # Navigation toolbar (zoom, pan, save)
        toolbar_frame = tk.Frame(wrapper, bg=BG_CARD)
        toolbar_frame.pack(fill=tk.X, padx=4, pady=(0, 6))
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        toolbar.update()

    def _on_analysis_error(self, msg):
        messagebox.showerror("Analysis Error", msg)
        self.status_var.set("Analysis failed")

    # ── Export ────────────────────────────────────────────────────────────────

    def _export_report(self):
        if self.df is None or self.metrics is None:
            messagebox.showwarning("No Results", "Run analysis first before exporting.")
            return

        try:
            report_dir, plots_dir, data_dir = fla.create_report_directory(self.csv_path)

            # Save all visible plots
            saved_plots = {}
            for widget in self.plot_frame.winfo_children():
                for child in widget.winfo_children():
                    if isinstance(child, tk.Widget):
                        canvas_widgets = [c for c in widget.winfo_children()
                                          if hasattr(c, 'figure')]
                        # FigureCanvasTkAgg wraps a tk widget
                        break

            # Use existing save function
            fig = fla.create_analysis_plots(self.df)
            if fig:
                saved_plots = fla.save_enhanced_plots(
                    fig, self.df, plots_dir, self.metrics,
                    self.integrity or fla.validate_data_integrity(self.df)
                )

            if self.beta_sweep_results:
                fla.plot_beta_sweep_results(self.beta_sweep_results, plots_dir)

            # Save report
            fla.save_comprehensive_report(
                self.csv_path, self.metrics, self.df,
                self.integrity or fla.validate_data_integrity(self.df),
                self.correlation_metrics or {},
                self.madgwick_analysis or {},
                report_dir, saved_plots
            )

            # Save data copy
            data_copy = os.path.join(data_dir, "original_data.csv")
            self.df.to_csv(data_copy, index=False)

            self.status_var.set(f"Report exported to {os.path.basename(report_dir)}")
            messagebox.showinfo("Export Complete", f"Report saved to:\n{report_dir}")

        except Exception as e:
            messagebox.showerror("Export Error", str(e))

    # ── Results formatters ───────────────────────────────────────────────────

    def _format_integrity(self, report):
        score = report.get("data_quality_score", 0)
        total = report.get("total_samples", 0)
        missing = len(report.get("missing_values", {}))
        oor = len(report.get("out_of_range_values", {}))
        gaps = len(report.get("timestamp_gaps", []))
        lines = [
            "═══ DATA INTEGRITY ═══",
            f"  Quality Score: {score:.1f}/100",
            f"  Total Samples: {total}",
            f"  Missing Values: {missing} cols",
            f"  Out-of-Range: {oor} cols",
            f"  Timestamp Gaps: {gaps}",
            ""
        ]
        return "\n".join(lines)

    def _format_stability(self, metrics):
        lines = ["═══ STABILITY METRICS ═══"]
        for axis in ["roll", "pitch", "yaw"]:
            if axis in metrics:
                m = metrics[axis]
                lines.append(f"  {axis.upper()}:")
                lines.append(f"    MAE:  {m['mae']:.4f}°")
                lines.append(f"    RMSE: {m['rmse']:.4f}°")
                lines.append(f"    Std:  {m['std']:.4f}°")
                lines.append(f"    SNR:  {m['snr_db']:.2f} dB")
                lines.append(f"    Drift: {m['drift_rate']:.6f}°/s")
                ct = m.get('convergence_time')
                lines.append(f"    Conv:  {ct:.3f}s" if ct else "    Conv:  N/A")
        lines.append("")
        return "\n".join(lines)

    def _format_correlation(self, corr):
        lines = ["═══ CROSS-CORRELATION ═══"]
        for axis in ["roll", "pitch", "yaw"]:
            key = f"attitude_{axis}_correlation"
            if key in corr:
                lines.append(f"  {axis.upper()}: {corr[key]:.4f}")
        lines.append("")
        return "\n".join(lines)

    def _format_beta_analysis(self, analysis):
        lines = ["═══ MADGWICK β ANALYSIS ═══"]
        if not analysis:
            lines.append("  No B_madgwick data")
        elif analysis.get("analysis_type") == "single_value":
            lines.append(f"  B_madgwick: {analysis.get('B_madgwick', '?'):.4f}")
            sa = analysis.get("stability_assessment", {})
            for axis in ["roll", "pitch", "yaw"]:
                if axis in sa:
                    lines.append(f"  {axis.upper()}: {sa[axis].get('stability_level', '?')}")
                    lines.append(f"    Rating: {sa[axis].get('performance_rating', '?')}")
        lines.append("")
        return "\n".join(lines)

    def _format_beta_sweep(self, sweep):
        lines = ["═══ β SWEEP RESULTS ═══"]
        opt = sweep.get("optimal", {})
        for axis in ["roll", "pitch", "yaw"]:
            if axis in opt:
                o = opt[axis]
                lines.append(f"  {axis.upper()}: β={o['beta']:.3f}")
                lines.append(f"    MAE={o['mae']:.3f}° RMSE={o['rmse']:.3f}°")
        lines.append("")
        return "\n".join(lines)

    def _format_event_log_summary(self, path):
        lines = ["═══ EVENT LOG (optional) ═══"]
        lines.append(f"  File: {os.path.basename(path)}")
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as f:
                content = f.readlines()
            events = [ln.strip() for ln in content if ln.strip() and not ln.startswith("#")]
            lines.append(f"  Events logged: {len(events)}")
            if events:
                lines.append(f"  First: {events[0][:72]}")
                if len(events) > 1:
                    lines.append(f"  Last:  {events[-1][:72]}")
        except OSError as e:
            lines.append(f"  Could not read: {e}")
        lines.append("")
        return "\n".join(lines)

    def _format_statistics(self, df):
        lines = ["═══ BASIC STATISTICS ═══"]
        for prefix, label in [("ctrl", "Control"), ("mon", "Monitor")]:
            lines.append(f"  {label} IMU:")
            for axis in ["roll", "pitch", "yaw"]:
                col = f"{prefix}_{axis}"
                if col in df.columns:
                    lines.append(f"    {axis}: μ={df[col].mean():.4f}° σ={df[col].std():.4f}°")
        if "err_roll" in df.columns:
            lines.append("  Errors (ctrl-mon):")
            for axis in ["roll", "pitch", "yaw"]:
                col = f"err_{axis}"
                lines.append(f"    {axis}: μ={df[col].mean():.4f}° |max|={df[col].abs().max():.4f}°")
        lines.append("")
        return "\n".join(lines)


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    app = FlightLogGUI()
    app.mainloop()

if __name__ == "__main__":
    main()
