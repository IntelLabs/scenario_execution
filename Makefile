file_finder = find . -type f $(1) -not \( -path './venv/*' -o -path './build/*' -o -path './log/*' -o -path './install/*' -o -path './dependencies/*' \)

PY_FILES = $(call file_finder,-name "*.py")
CPP_FILES = $(call file_finder,-name "*.cpp")
H_FILES = $(call file_finder,-name "*.cpp")
C_FILES = $(call file_finder,-name "*.cpp")

LINKCHECKDIR  = build/linkcheck

check: check_format pylint

format:
	$(PY_FILES) | xargs autopep8 --in-place --max-line-length=140
	$(CPP_FILES) | xargs clang-format -i
	$(H_FILES) | xargs clang-format -i
	$(C_FILES) | xargs clang-format -i

check_format:
	$(PY_FILES) | xargs autopep8 --diff --max-line-length=140 --exit-code

pylint:
	$(PY_FILES) | xargs pylint --rcfile=.github/linters/.pylintrc

sphinx_setup:
	if [ ! -d "venv" ]; then \
		python -m venv venv/; \
		. venv/bin/activate; \
		pip install -r Docs/requirements.txt; \
		deactivate; \
	fi

doc: sphinx_setup checklinks checkspelling
	. venv/bin/activate && GITHUB_REF_NAME=local GITHUB_REPOSITORY=intel-innersource/applications.robotics.mobile.scenario-execution python -m sphinx -b html -W Docs build/html

view_doc: doc
	firefox build/html/index.html &

checklinks: sphinx_setup
	. venv/bin/activate && GITHUB_REF_NAME=local GITHUB_REPOSITORY=intel-innersource/applications.robotics.mobile.scenario-execution python -m sphinx -b html -b linkcheck -W Docs $(ALLSPHINXOPTS) $(LINKCHECKDIR)
	@echo
	@echo "Check finished. Report is in $(LINKCHECKDIR)."

checkspelling: sphinx_setup
	. venv/bin/activate && GITHUB_REF_NAME=local GITHUB_REPOSITORY=intel-innersource/applications.robotics.mobile.scenario-execution python -m sphinx -b html -b spelling -W Docs $(ALLSPHINXOPTS) $(LINKCHECKDIR)
	@echo
	@echo "Check finished. Report is in $(LINKCHECKDIR)."
