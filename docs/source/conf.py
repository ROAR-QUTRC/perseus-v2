import shutil

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "Perseus V2"
copyright = "2024, ROAR Team"
author = "ROAR Team"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["myst_parser", "breathe", "exhale"]

templates_path = ["_templates"]
exclude_patterns = []

language = "en"

# -- Options for Breathe -----------------------------------------------------
# https://breathe.readthedocs.io/en/stable/quickstart.html

breathe_projects = {"rover": "../generated/xml/"}
breathe_default_project = "rover"

# Setup the exhale extension
exhale_args = {
    # Required arguments
    "containmentFolder": "./generated",
    "rootFileName": "root.rst",
    "rootFileTitle": "Code Documentation",
    "doxygenStripFromPath": "..",
    # Optional arguments
    "createTreeView": True,
    # configure Exhale to run doxygen (if it's available)
    "exhaleExecutesDoxygen": (shutil.which("doxygen") is not None),
    # input here is technically unnecessary since we're providing a Doxygen config file, but Exhale requires it
    "exhaleDoxygenStdin": "../Doxyfile-prj.cfg INPUT=./../../software",
    # Page layout configuration
    "contentsDirectives": False,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
}

# -- Options for MyST parser -------------------------------------------------
# https://myst-parser.readthedocs.io/en/latest/
myst_heading_anchors = 4  # auto-generated heading anchors (slugs)

# -- Options for markup ------------------------------------------------------

primary_domain = "cpp"
highlight_language = "cpp"

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

use_rtd_theme = False
html_theme = "sphinx_rtd_theme" if use_rtd_theme else "furo"
html_static_path = ["_static"]

html_extra_path = ["robots.txt"]
