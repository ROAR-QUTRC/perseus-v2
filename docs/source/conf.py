import os
from os import path
from shutil import which
from textwrap import dedent
from exhale import utils

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# Project information
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "Perseus V2"
author = "ROAR Team"
copyright = f"%Y, {author}"

# General configuration
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",  # MyST parser - allows for markdown instead of reStructuredText
    "breathe",
    "exhale",  # doxygen integration
    "sphinxcontrib.jquery",  # add jQuery to the HTML output so that plugins expecting to run on RTD work
    "sphinx.ext.githubpages",  # add .nojekyll file for github pages
    "sphinx.ext.intersphinx",  # link to other projects (specifically ROS and Python)
    "sphinx.ext.todo",  # enable todo directives
    "sphinx_immaterial",  # theme
    "sphinx_immaterial.kbd_keys",  # pretty keyboard shortcuts
    "sphinx_immaterial.apidoc.cpp.cppreference",  # link to en.cppreference.com
    "sphinx_immaterial.graphviz",  # graphviz support with theme integration
    "sphinx_tippy",  # preview tooltips on link hover
]

manpages_url = "https://manpages.debian.org/{path}"

templates_path = ["_templates"]
exclude_patterns = []

language = "en"

# Extension configuration
# Set up the breathe extension
# https://breathe.readthedocs.io/en/stable/quickstart.html
doxygen_xml_dir = "../build/doxygen/xml"
breathe_projects = {"rover": doxygen_xml_dir}
breathe_default_project = "rover"


# Setup the exhale extension
# https://exhale.readthedocs.io/en/stable/usage.html
# this needs to be overridden so that we can enable graphs
def exhale_kind_overrides(kind):
    """
    For a given input ``kind``, return the list of reStructuredText specifications
    for the associated Breathe directive.
    """
    # override doxygen class and struct directives to include graphs
    if kind == "class" or kind == "struct":
        return [
            ":members:",
            ":protected-members:",
            ":undoc-members:",
            ":allow-dot-graphs:",
        ]
    # An empty list signals to Exhale to use the defaults
    else:
        return []


exhale_args = {
    # Required arguments
    "containmentFolder": "./generated/exhale",
    "rootFileName": "index.rst",
    "rootFileTitle": "Generated Documentation",
    "doxygenStripFromPath": "../..",
    # Optional arguments
    "createTreeView": True,
    # configure Exhale to run doxygen if it's available AND we haven't already generated output (speeds up dev shell builds a bit)
    "exhaleExecutesDoxygen": (
        (which("doxygen") is not None) and not os.path.isdir(doxygen_xml_dir)
    ),
    # dedent so we can provide multiline string
    "exhaleDoxygenStdin": dedent(
        """
        INPUT= ./../../software ./../../firmware/components
        RECURSIVE = YES
        EXCLUDE_PATTERNS = */tests/* */test/* */launch/*
        EXCLUDE_PATTERNS += */build/*
        EXCLUDE_PATTERNS += *.py __pycache__*
        EXCLUDE_PATTERNS += */main.cpp
        EXCLUDE_PATTERNS += *.md
        # packages that don't need documentation (most of them)
        # and are problematic to generate docs for (a few of them)
        EXCLUDE_PATTERNS += */src/perseus_bt_nodes/* */src/perseus_hardware/*
        EXCLUDE_SYMLINKS = YES

        BUILTIN_STL_SUPPORT = YES

        # generate graphs
        HAVE_DOT = YES

        # exclude main functions
        EXCLUDE_SYMBOLS += main

        PREDEFINED += DOXYGEN_SHOULD_SKIP_THIS
        SKIP_FUNCTION_MACROS = YES
        """
    ),
    # Page layout configuration
    "contentsDirectives": False,
    "pageLevelConfigMeta": dedent(
        """
        .. highlight:: none

        """
    ),
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    "customSpecificationsMapping": utils.makeCustomSpecificationsMapping(
        exhale_kind_overrides
    ),
}

# set up MyST parser extension
# https://myst-parser.readthedocs.io/en/latest/
myst_enable_extensions = [
    "amsmath",
    "attrs_block",
    "attrs_inline",
    "colon_fence",
    "deflist",
    "dollarmath",
    "html_admonition",
    "html_image",
    "linkify",
    "replacements",
    "strikethrough",
]
myst_heading_anchors = 4  # auto-generated heading anchors (slugs)
suppress_warnings = ["myst.strikethrough"]

ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
# intersphinx config
# If you're adding a site to this list, you also need to add it to the Github Actions workflow
# to allow intersphinx to access the site during a build
# this is a good guide: https://docs.readthedocs.io/en/stable/guides/intersphinx.html
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "ros": (f"https://docs.ros.org/en/{ros_distro}", None),
    "sphinx": ("https://www.sphinx-doc.org/en/master", None),
    "myst": (
        "https://myst-parser.readthedocs.io/en/latest",
        None,
    ),
    "breathe": (
        "https://breathe.readthedocs.io/en/stable",
        None,
    ),
    "exhale": ("https://exhale.readthedocs.io/en/stable", None),
    "sphinx-immaterial": (
        "https://jbms.github.io/sphinx-immaterial",
        None,
    ),
}
# Sphinx defaults to automatically resolve *unresolved* labels using all your Intersphinx mappings.
# This behavior has unintended side-effects, namely that documentations local references can
# suddenly resolve to an external location.
# See also:
# https://www.sphinx-doc.org/en/master/usage/extensions/intersphinx.html#confval-intersphinx_disabled_reftypes
intersphinx_disabled_reftypes = ["*"]

todo_include_todos = True

# Options for markup
primary_domain = "cpp"
rst_prolog = """
.. role:: py-inline(code)
   :language: python

.. role:: cpp-inline(code)
   :language: cpp
"""
highlight_language = "cpp"

# Options for HTML output
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
html_theme = "sphinx_immaterial"
html_static_path = ["_static"]

html_extra_path = ["robots.txt", "README.md"]

html_css_files = [
    "css/theme.css",
]
html_js_files = [
    "js/dark-opt-images.js",
]

html_logo = "_static/Rover-Logo.svg"
html_favicon = "_static/Logo-Simple.png"

primary = "deep-orange"
accent = "indigo"

# sphinx-immaterial theme options
html_theme_options = {
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
    # using self-hosted fonts (Roboto): See _static/css/fonts.css and https://sphinx-immaterial.readthedocs.io/en/latest/customization.html#themeconf-font
    # This prevents CDN downloads and speeds up the build
    "font": False,
    "globaltoc_collapse": False,
    "toc_title_is_page_title": True,
}
# sphinx_rtd_theme config - no longer used
# {
#     "collapse_navigation": False,
#     "sticky_navigation": True,
#     "navigation_depth": -1,
# }

# part of sphinx-immaterial: custom objects
# https://jbms.github.io/sphinx-immaterial/apidoc/index.html
object_description_options = [
    ("std:confval", dict(toc_icon_class="data", toc_icon_text="C")),
    ("std:option", dict(toc_icon_class="sub-data", toc_icon_text="O")),
]

# markup options
primary_domain = "cpp"

# Output configuration/extensions/Sphinx python configuration
nitpicky = True
nitpick_ignore_regex = {
    # not being able to find STL documentation
    ("cpp:identifier", "std.*"),
    # or ROS things
    ("cpp:identifier", "rclcpp.*"),
    # silence {expr} role namespace warnings
    ("cpp:identifier", "hi_can.*"),
    ("cpp:identifier", "addressing.*"),
    ("cpp:identifier", "parameters.*"),
}

source_suffix = {
    ".rst": "restructuredtext",
    ".txt": "markdown",
    ".md": "markdown",
}

suppress_warnings = [
    # unfortunately Exhale generates a LOT of "could not find ..." docutils warnings,
    # so we need to silence it :(
    "docutils",
    # and tippy Wikipedia references fail for some reason
    "tippy.wiki",
]


def index_figures(app):
    INDEX_FILE = "_figure-index.rst"
    figure_dir = path.join(app.builder.srcdir, "generated")

    # only run if the directory exists
    if not path.exists(figure_dir):
        return

    # use scandir to get all files in the directory - see:
    # https://stackoverflow.com/questions/800197/how-to-get-all-of-the-immediate-subdirectories-in-python
    figure_paths = [
        f.name
        for f in os.scandir(figure_dir)
        if (
            f.is_file()
            and (
                f.name.endswith(".svg")
                or f.name.endswith(".png")
                or f.name.endswith(".jpg")
                or f.name.endswith(".jpeg")
                or f.name.endswith(".pdf")
            )
        )
    ]

    # write the index file which embeds the image
    file = open(path.join(figure_dir, INDEX_FILE), "w")
    file.write(
        ":orphan:\n\n"  # suppress warnings about not being included in toctree
        "Generated Images\n============================\n\n"
        "This is a dirty hack to ensure that Sphinx includes all the generated figures in the output.\n\n"
    )
    for image in figure_paths:
        file.write(f".. image:: {image}\n")


def add_graph_fix_css(app, pagename, templatename, context, doctree):
    if pagename.startswith("generated/"):
        app.add_css_file("css/graph-fix.css")


# silence missing reference warnings (mostly namespaces) from generated code
def silence_warnings(app, domain, node):
    is_cpp = domain.name == "cpp" or domain.name == "std"
    if not is_cpp:
        return False

    is_generated = node["refdoc"].startswith("generated/exhale")
    if is_generated:
        return True


def setup(app):
    # run the figures hack
    app.connect("builder-inited", index_figures)
    # add graph fix CSS to generated documentation
    # Fixes background fill colours on the "this class" node in class/collaboration diagrams
    app.connect("html-page-context", add_graph_fix_css)

    app.connect("warn-missing-reference", silence_warnings)

    # add custom confval and option objects
    app.add_object_type(
        "confval",
        "confval",
        objname="configuration value",
        indextemplate="pair: %s; configuration value",
    )
    app.add_object_type(
        "option",
        "option",
        objname="command-line option",
        indextemplate="pair: %s; command-line option",
    )
