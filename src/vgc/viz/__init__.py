"""Visualization exports with lazy loading for lightweight live telemetry."""

__all__ = [
    "export_run_figures",
    "export_suite_comparison_figures",
]


def __getattr__(name):
    if name in {"export_run_figures", "export_suite_comparison_figures"}:
        from .paper_figures import export_run_figures, export_suite_comparison_figures

        return {
            "export_run_figures": export_run_figures,
            "export_suite_comparison_figures": export_suite_comparison_figures,
        }[name]
    raise AttributeError(name)
