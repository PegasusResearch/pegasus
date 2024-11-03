# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
# This section is responsible for auto-generating the API documentation
import os
import sys

# -- Project information -----------------------------------------------------

project = "Pegasus"
copyright = "2024, Marcelo Jacinto"
author = "Marcelo Jacinto"
version = "1.0.0"

# -- General configuration ---------------------------------------------------

extensions = [
    "sphinx.ext.duration",
    "sphinx.ext.doctest",
    "sphinx.ext.autodoc",
    "autodocsumm",
    'sphinx.ext.napoleon',
    #"sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
    "myst_parser",
    "sphinx.ext.mathjax",
    "sphinxcontrib.bibtex",
    "sphinx.ext.todo",
    "sphinx.ext.githubpages",
    "sphinx.ext.autosectionlabel",
    "sphinxcontrib.youtube",
    "sphinxcontrib.mermaid",
    "myst_parser"
]

intersphinx_mapping = {
    "rtd": ("https://docs.readthedocs.io/en/stable/", None),
    "python": ("https://docs.python.org/3/", None),
    "sphinx": ("https://www.sphinx-doc.org/en/master/", None),
}

# mathjax hacks
mathjax3_config = {
    "tex": {
        "inlineMath": [["\\(", "\\)"]],
        "displayMath": [["\\[", "\\]"]],
    },
}

intersphinx_disabled_domains = ["std"]

# supported file extensions for source files
#source_suffix = {
#    ".rst": "restructuredtext"
#}

templates_path = ["_templates"]

suppress_warnings = ["myst.header", "autosectionlabel.*"]

# -- Options for EPUB output
epub_show_urls = "footnote"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "README.md", "licenses/*"]

# --- Automatic API documentation generation

# put type hints inside the description instead of the signature (easier to read)
autodoc_typehints = "description"
autodoc_typehints_description_target = "documented"
# document class *and* __init__ methods
autoclass_content = "class"  #
# separate class docstring from __init__ docstring
autodoc_class_signature = "separated"
# sort members by source order
autodoc_member_order = "groupwise"
# default autodoc settings
autodoc_default_options = {
    "autosummary": True,
}

# BibTeX configuration
bibtex_bibfiles = ["bibliography.bib"]

# Generate documentation for __special__ methods
napoleon_include_special_with_doc = True

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]
html_logo = "_static/logo.png"
html_theme_options = {
    'logo_only': True,
    'display_version': False,
    'style_nav_header_background': '#000000'
}
html_css_files = [
    'custom.css',
]

html_show_copyright = True
html_show_sphinx = False

# The master toctree document.
master_doc = "index"