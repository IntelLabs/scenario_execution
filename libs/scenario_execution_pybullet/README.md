# Scenario Execution Library for PyBullet

The `scenario_execution_pybullet` package provides actions to interact with the [PyBullet simulator](https://pybullet.org/).

It provides the following scenario execution library:

- `pybullet.osc`: PyBullet specific actions to interact with the simulation


## Example

To run the example, first build the `scenario_execution_pybullet` package and its dependencies:

```bash
python -m venv my_venv
source my_venv/bin/activate
pip install -r libs/scenario_execution_pybullet/requirements.txt
pip install -e libs/scenario_execution_pybullet libs/scenario_execution
```

Now, run the following command to launch the scenario:

```bash
./my_venv/lib/scenario_execution/scenario_execution libs/scenario_execution_pybullet/scenario_execution_pybullet/scenarios/example_pybullet.osc -s 0.00416666666 -t
```
