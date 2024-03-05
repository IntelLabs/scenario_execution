# pylint: disable=all

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import os
import datetime

project = "Scenario Execution"
copyright = f"{datetime.datetime.now()}, Intel"
author = "Intel"

version = '1.0.0'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx.ext.extlinks',
              'sphinxcontrib.spelling']

extlinks = {'repo_link': ('https://github.com/intellabs/scenario_execution/blob/main/%s', '%s')}

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

language = 'english'

linkcheck_ignore = [
    r'https://github.com/intellabs/scenario_execution/.*',
]

spelling_word_list_filename = 'dictionary.txt'
spelling_ignore_contributor_names=False

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
# html_static_path = ['.']

# https://docs.github.com/en/actions/learn-github-actions/contexts#github-context
github_user, github_repo = os.environ["GITHUB_REPOSITORY"].split("/", maxsplit=1)
html_context = {
    'display_github': True,
    'github_user': github_user,
    'github_repo': github_repo,
    'github_version': os.environ["GITHUB_REF_NAME"] + '/docs/',
}
