file_finder = find . -type f $(1) -not \( -path './venv/*' -o -path './build/*' -o -path './log/*' -o -path './install/*' -o -path './dependencies/*' \)

PY_FILES = $(call file_finder,-name "*.py")
CPP_FILES = $(call file_finder,-name "*.cpp")
H_FILES = $(call file_finder,-name "*.h")

LINKCHECKDIR  = build/linkcheck

check: check_format pylint

format:
	$(PY_FILES) | xargs autopep8 --in-place --max-line-length=140
	$(CPP_FILES) | xargs clang-format -i
	$(H_FILES) | xargs clang-format -i

check_format:
	$(PY_FILES) | xargs autopep8 --diff --max-line-length=140 --exit-code

pylint:
	$(PY_FILES) | xargs pylint --rcfile=.github/linters/.pylintrc

sphinx_setup:
	if [ ! -d "venv" ]; then \
		python -m venv venv/; \
		. venv/bin/activate; \
		pip install -r docs/requirements.txt; \
		deactivate; \
	fi

doc: sphinx_setup checklinks checkspelling
	. venv/bin/activate && GITHUB_REF_NAME=local GITHUB_REPOSITORY=intellabs/scenario_execution python -m sphinx -b html -W docs build/html

view_doc: doc
	firefox build/html/index.html &

checklinks: sphinx_setup
	. venv/bin/activate && GITHUB_REF_NAME=local GITHUB_REPOSITORY=intellabs/scenario_execution python -m sphinx -b html -b linkcheck -W docs $(ALLSPHINXOPTS) $(LINKCHECKDIR)
	@echo
	@echo "Check finished. Report is in $(LINKCHECKDIR)."

checkspelling: sphinx_setup
	. venv/bin/activate && GITHUB_REF_NAME=local GITHUB_REPOSITORY=intellabs/scenario_execution python -m sphinx -b html -b spelling -W docs $(ALLSPHINXOPTS) $(LINKCHECKDIR)
	@echo
	@echo "Check finished. Report is in $(LINKCHECKDIR)."

test_scenario_execution_nav2_test:
	scenario_batch_execution -i test/scenario_execution_nav2_test/scenarios/ -o test_scenario_execution_nav2 --ignore-process-return-value -- ros2 run scenario_execution_ros scenario_execution_ros {SCENARIO} -o {OUTPUT_DIR} -t

test_scenario_execution_gazebo_test:
	scenario_batch_execution -i test/scenario_execution_gazebo_test/scenarios/ -o test_scenario_execution_gazebo --ignore-process-return-value -- ros2 launch tb4_sim_scenario sim_nav_scenario_launch.py scenario:={SCENARIO} output_dir:={OUTPUT_DIR} headless:=True use_rviz:=False navigation:=false