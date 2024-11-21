import shutil
from textwrap import dedent

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# Project information
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "Perseus V2"
copyright = "2024, ROAR Team"
author = "ROAR Team"

# may be: "sphinx_immaterial", "sphinx_rtd_theme" or "furo"
theme = "sphinx_immaterial"

# General configuration
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",  # MyST parser - allows for markdown instead of reStructuredText
    "breathe",
    "exhale",  # doxygen integration
    "sphinxcontrib.jquery",  # add jQuery to the HTML output so that plugins expecting to run on RTD work
    "sphinx.ext.githubpages",  # add .nojekyll file for github pages
    "sphinx_immaterial.kbd_keys",  # pretty keyboard shortcuts
    "sphinx.ext.intersphinx",  # link to other projects (specifically ROS and Python)
]
if theme == "sphinx_immaterial":
    extensions.append("sphinx_immaterial")

templates_path = ["_templates"]
exclude_patterns = []

language = "en"

# Extension configuration
# Set up the breathe extension
# https://breathe.readthedocs.io/en/stable/quickstart.html
breathe_projects = {"rover": "../build/doxygen/xml/"}
breathe_default_project = "rover"

# Setup the exhale extension
exhale_args = {
    # Required arguments
    "containmentFolder": "./generated",
    "rootFileName": "index.rst",
    "rootFileTitle": "Generated Documentation",
    "doxygenStripFromPath": "../../software",
    # Optional arguments
    "createTreeView": True,
    # configure Exhale to run doxygen (if it's available)
    "exhaleExecutesDoxygen": (shutil.which("doxygen") is not None),
    # dedent so we can provide multiline string
    "exhaleDoxygenStdin": dedent(
        """
        INPUT=./../../software
        RECURSIVE = YES
        EXCLUDE_PATTERNS = *.md setup.py __init__.py __pycache__* */tests/* */test/*
        EXCLUDE_SYMLINKS = YES

        # we do NOT want program listings as that exposes the source code
        XML_PROGRAMLISTING = NO
        """
    ),
    # Page layout configuration
    "contentsDirectives": False,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
}

# set up MyST parser extension
# https://myst-parser.readthedocs.io/en/latest/
myst_enable_extensions = [
    "amsmath",
    "attrs_inline",
    "colon_fence",
    "dollarmath",
    "html_admonition",
    "html_image",
    "linkify",
    "replacements",
    "strikethrough",
]
myst_heading_anchors = 4  # auto-generated heading anchors (slugs)
suppress_warnings = ["myst.strikethrough"]

# intersphinx config
# this is a good guide: https://docs.readthedocs.io/en/stable/guides/intersphinx.html
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "ros": ("https://docs.ros.org/en/humble", None),
}

# Options for markup
primary_domain = "cpp"
highlight_language = "cpp"

# Options for HTML output
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
html_theme = theme
html_static_path = ["_static"]

html_extra_path = ["robots.txt", "README.md"]

html_css_files = [
    "css/theme.css",
]

html_logo = "_static/Rover-Logo.svg"
html_favicon = "_static/Logo-Simple.png"

primary = "deep-orange"
accent = "indigo"

# sphinx-immaterial theme options
html_theme_options = (
    {
        "repo_url": "https://github.com/ROAR-QUTRC/perseus-v2",
        "repo_name": "Perseus V2",
        "icon": {
            "repo": "material/github",
            "edit": "material/file-edit-outline",
        },
        "edit_uri": "edit/main/docs/source/",
        "features": [
            # "content.tabs.link", # linked tabs which all switch at once, eg for different languages
            # "header.autohide", # autohide header on scroll
            # "navigation.expand", # expand navigation by default (on load)
            "navigation.instant",  # instant navigation
            "navigation.sections",  # split navigation into sections (by caption mostly)
            # "navigation.tabs",
            # "navigation.tabs.sticky",
            "navigation.top",  # back to top button
            # "navigation.tracking", # URL tracks header
            "search.highlight",  # highlight search results
            "search.share",  # share search results button
            # "toc.integrate", # integrate ToC with left navigation
            "toc.follow",  # ToC active header follows scroll
            "toc.sticky",  # sticky ToC header
        ],
        "palette": [
            {
                "media": "(prefers-color-scheme: light)",
                "scheme": "default",
                "primary": primary,
                "accent": accent,
                "toggle": {
                    "icon": "material/lightbulb-outline",
                    "name": "Switch to dark mode",
                },
            },
            {
                "media": "(prefers-color-scheme: dark)",
                "scheme": "slate",
                "primary": primary,
                "accent": accent,
                "toggle": {
                    "icon": "material/lightbulb",
                    "name": "Switch to light mode",
                },
            },
        ],
        "globaltoc_collapse": False,
        "toc_title_is_page_title": True,
    }
    if (theme == "sphinx_immaterial")
    else (
        {
            "collapse_navigation": False,
            "sticky_navigation": True,
            "navigation_depth": -1,
        }
        if theme == "sphinx_rtd_theme"
        else {}
    )
)
