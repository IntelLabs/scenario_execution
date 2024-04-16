import subprocess

node_process = subprocess.Popen(['ros2', 'launch', 'scenario_execution_control', 'scenario_execution_control_launch.py',
                                'scenario_dir:=scenario_execution_control/test/scenarios/', 'output_dir:=test_example_control'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

test_service_call = subprocess.run(['python3', 'scenario_execution_control/test/scenario_execution_control_test.py',
                                   'scenario_execution_control/test/scenario/scenario_execution_control_test.osc'], check=True)

exit_status = test_service_call.returncode

if exit_status == 0:
    print('Test passed successfully.')
else:
    print("Test failed")

node_process.terminate()
node_process.wait()
