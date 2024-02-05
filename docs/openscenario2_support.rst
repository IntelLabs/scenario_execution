OpenSCENARIO2 Support
---------------------

This tool supports a subset of the `OpenSCENARIO
2 <https://www.asam.net/project-detail/asam-openscenario-v20-1/>`__ PRC
standard. In the following the supported features are described.

In addition, it is recommended to take a look into the official
documentation available
`here <https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/welcome.html>`__.

The `standard library of
OSC2 <https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/domain-model/standard_library.html>`__
was slightly adapted as py-osc2 is not yet support multiline enum
definitions.

Level of support
~~~~~~~~~~~~~~~~

In the following the OpenSCENARIO2 standard library elements are listed
with their current support status.

Scalar types and units
^^^^^^^^^^^^^^^^^^^^^^

.. raw:: html

   <!-- True: &#9989; -->

.. raw:: html

   <!-- False: &#10060; -->

========================================= ======= =====
Element Tag                               Support Notes
========================================= ======= =====
``library-physical-length``               ✅       
``library-physical-time``                 ✅       
``library-physical-acceleration``         ✅       
``library-physical-jerk``                 ✅       
``library-physical-angle``                ✅       
``library-physical-speed``                ✅       
``library-physical-angular_rate``         ✅       
``library-physical-angular_acceleration`` ✅       
``library-physical-mass``                 ✅       
``library-physical-temperature``          ✅       
``library-physical-pressure``             ✅       
``library-physical-luminous_intensity``   ✅       
``library-physical-luminous_flux``        ✅       
``library-physical-illuminance``          ✅       
``library-physical-electrical_current``   ✅       
``library-physical-amount_of_substance``  ✅       
========================================= ======= =====

Structs
^^^^^^^

========================================= ======= ========
Element Tag                               Support Notes
========================================= ======= ========
``library-position_3d``                   ✅       
``library-celestial_position_2d``         ✅       
``library-geodetic_position_2d``          ✅       
``library-orientation_3d``                ✅       
``library-pose_3d``                       ✅       
``library-translational_velocity_3d``     ✅       
``library-orientation_rate_3d``           ✅       
``library-velocity_6d``                   ✅       
``library-translational_acceleration_3d`` ✅       
``library-orientation_acceleration_3d``   ✅       
``library-acceleration_6d``               ✅       
``axle``                                  ✅       
``bounding_box``                          ✅       
``crossing_type``                         ✅       
``route_point``                           ✅       
``xyz_point``                             ✅       
``odr_point``                             ✅       
``library-path``                          ✅       modified
``relative_path``                         ✅       
``relative_path_pose_3d``                 ✅       
``relative_path_st``                      ✅       
``relative_path_odr``                     ✅       
``trajectory``                            ✅       
``relative_trajectory``                   ✅       
``relative_trajectory_pose_3d``           ✅       
``relative_trajectory_st``                ✅       
``relative_trajectory_odr``               ✅       
``any_shape``                             ✅       
``any_acceleration_shape``                ✅       
``any_speed_shape``                       ✅       
``any_position_shape``                    ✅       
``any_lateral_shape``                     ✅       
``common_acceleration_shape``             ✅       
``common_speed_shape``                    ✅       
``common_position_shape``                 ✅       
``common_lateral_shape``                  ✅       
``behavioral_model``                      ✅       
``bm_engine``                             ✅       
========================================= ======= ========

Enums
^^^^^

=========================== ======= ========
Element Tag                 Support Notes
=========================== ======= ========
``color``                   ✅       
``intended_infrastructure`` ✅       
``vehicle_category``        ✅       
``driving_rule``            ✅       
``directionality``          ✅       
``lane_type``               ✅       
``lane_use``                ✅       
``side_left_right``         ✅       
``crossing_marking``        ✅       modified
``crossing_use``            ✅       
``crossing_elevation``      ✅       
``junction_direction``      ✅       
``route_overlap_kind``      ✅       
``lateral_overlap_kind``    ✅       
``dynamic_profile``         ✅       
``lane_change_side``        ✅       
``gap_direction``           ✅       
``headway_direction``       ✅       
``lat_measure_by``          ✅       
``yaw_measure_by``          ✅       
``orientation_measured_by`` ✅       
``movement_options``        ✅       
``connect_route_points``    ✅       modified
``path_interpolation``      ✅       
``at``                      ✅       
``movement_mode``           ✅       
``track``                   ✅       
``distance_direction``      ✅       
``distance_mode``           ✅       
``relative_transform``      ✅       
``on_route_type``           ✅       
``route_distance_enum``     ✅       
=========================== ======= ========

Actor
^^^^^

======================= ======= ========
Element Tag             Support Notes
======================= ======= ========
``osc_actor``           ✅       
``physical_object``     ✅       modified
``stationary_object``   ✅       
``movable_object``      ✅       modified
``traffic_participant`` ✅       
``vehicle``             ✅       
``person``              ✅       
``animal``              ✅       
======================= ======= ========

Environment (An Actor)
^^^^^^^^^^^^^^^^^^^^^^

========================== ======= ========
Element Tag                Support Notes
========================== ======= ========
``environment``            ✅       modified
``weather``                ✅       
``air``                    ✅       
``precipitation``          ✅       
``wind``                   ✅       
``fog``                    ✅       
``clouds``                 ✅       
``celestial_light_source`` ✅       
========================== ======= ========

Map (An Actor)
^^^^^^^^^^^^^^

================== ======= =====
Element Tag        Support Notes
================== ======= =====
``map``            ✅       
``route``          ✅       
``route_element``  ✅       
``road``           ✅       
``lane_section``   ✅       
``lane``           ✅       
``crossing``       ✅       
``junction``       ✅       
``compound_route`` ✅       
``compound_lane``  ✅       
================== ======= =====

Action - movable object
^^^^^^^^^^^^^^^^^^^^^^^

============================================ ======= =====
Element Tag                                  Support Notes
============================================ ======= =====
``osc_actor.osc_action``                     ✅       
``movable_object.action_for_movable_object`` ✅       
``movable_object.move``                      ✅       
``movable_object.remain_stationary``         ✅       
``movable_object.assign_position``           ✅       
``movable_object.assign_orientation``        ✅       
``movable_object.assign_speed``              ✅       
``movable_object.assign_acceleration``       ✅       
``movable_object.replay_path``               ✅       
``movable_object.replay_trajectory``         ✅       
``movable_object.change_position``           ✅       
``movable_object.change_speed``              ✅       
``movable_object.keep_speed``                ✅       
``movable_object.change_acceleration``       ✅       
``movable_object.keep_acceleration``         ✅       
``movable_object.follow_path``               ✅       
``movable_object.follow_trajectory``         ✅       
============================================ ======= =====

Action - vehicle
^^^^^^^^^^^^^^^^

=============================== ======= =====
Element Tag                     Support Notes
=============================== ======= =====
``vehicle.action_for_vehicle``  ✅       
``vehicle.drive``               ✅       
``vehicle.follow_lane``         ✅       
``vehicle.change_lane``         ✅       
``vehicle.change_space_gap``    ✅       
``vehicle.keep_space_gap``      ✅       
``vehicle.change_time_headway`` ✅       
``vehicle.keep_time_headway``   ✅       
=============================== ======= =====

Action - person
^^^^^^^^^^^^^^^

============================ ======= =====
Element Tag                  Support Notes
============================ ======= =====
``person.action_for_person`` ✅       
``person.walk``              ✅       
============================ ======= =====

Action - environment
^^^^^^^^^^^^^^^^^^^^

========================================= ======= =====
Element Tag                               Support Notes
========================================= ======= =====
``environment.action_for_environment``    ✅       
``environment.air``                       ✅       
``environment.rain``                      ✅       
``environment.snow``                      ✅       
``environment.wind``                      ✅       
``environment.fog``                       ✅       
``environment.cloud``                     ✅       
``environment.assign_celestial_position`` ✅       
========================================= ======= =====

Modifier - location based modifiers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

============================== ======= =====
Element Tag                    Support Notes
============================== ======= =====
``position``                   ❌       
``distance``                   ❌       
``lane``                       ❌       
``keep_lane``                  ❌       
``lateral``                    ❌       
``yaw``                        ❌       
``orientation``                ❌       
``along``                      ❌       
``keep_position``              ❌       
``along_trajectory``           ❌       
``stationary_object.location`` ❌       
============================== ======= =====

Modifier - rate of change based modifiers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

===================== ======= =====
Element Tag           Support Notes
===================== ======= =====
``speed``             ❌       
``acceleration``      ❌       
``keep_speed``        ❌       
``change_speed``      ❌       
``physical_movement`` ❌       
``avoid_collisions``  ❌       
``change_lane``       ❌       
===================== ======= =====

Modifier - map based modifiers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

================================ ======= =====
Element Tag                      Support Notes
================================ ======= =====
``map.number_of_lanes``          ❌       
``map.routes_are_in_sequence``   ❌       
``map.roads_follow_in_junction`` ❌       
``map.routes_overlap``           ❌       
``map.lane_side``                ❌       
``map.compound_lane_side``       ❌       
``map.end_lane``                 ❌       
``map.start_lane``               ❌       
``map.crossing_connects``        ❌       
``map.routes_are_opposite``      ❌       
``map.set_map_file``             ❌       
================================ ======= =====
