"""Visualization exports with lazy loading for lightweight live telemetry."""

__all__ = [
    "export_run_figures",
    "export_suite_comparison_figures",
    "derive_diagnostics",
    "slice_release_window",
]


def __getattr__(name):
    if name in {"export_run_figures", "export_suite_comparison_figures"}:
        from .paper_figures import export_run_figures, export_suite_comparison_figures

        return {
            "export_run_figures": export_run_figures,
            "export_suite_comparison_figures": export_suite_comparison_figures,
        }[name]
    if name in {"derive_diagnostics", "slice_release_window"}:
        from .diagnostics import derive_diagnostics, slice_release_window

        return {
            "derive_diagnostics": derive_diagnostics,
            "slice_release_window": slice_release_window,
        }[name]
    raise AttributeError(name)
