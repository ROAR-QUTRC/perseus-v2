# Configuration file for the Sphinx documentation builder.

project = "AS7343 Spectral Sensor Driver"
copyright = "2026, Perseus Rover Team"
author = "Perseus Rover Team"
release = "0.0.1"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme = "alabaster"
html_static_path = ["_static"]
