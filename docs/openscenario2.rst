OpenSCENARIO DSL
================

General
-------

This tool supports a subset of the `OpenSCENARIO DSL <https://www.asam.net/standards/detail/openscenario-dsl/>`__ standard.

The official documentation is available
`here <https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_DSL/latest/index.html>`__.

The `standard library of
OSC2 <https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_DSL/latest/domain-model/_attachments/ASAM_OpenSCENARIO_DSL_v2.1.0_Domain_model_library.zip>`__
was adapted to be usable by the current parsing support of scenario execution.


Mapping to py-trees
-------------------

.. list-table:: 
   :widths: 15 25 60
   :header-rows: 1
   :class: tight-table   
   
   - * OpenScenario2  
     * py-trees
     * Comment
   - * ``action``
     * ``Behaviour``
     * Actions are derived from ``scenario_execution.actions.base_action.BaseAction`` which is derived from ``py_trees.behaviour.Behaviour``
   - * ``event``
     * blackboard entry and ``Behaviour``
     * ``Behaviour`` is used to read and write blackboard variable
   - * ``modifier``
     * ``Decorator``
     *
   - * ``var``
     * blackboard entry
     * Variables are stored within the blackboard


.. role:: raw-html(raw)
   :format: html

Supported features
------------------

In the following the OpenSCENARIO DSL keywords are listed with their current support status.


======================= ==================== =============================
Element Tag             Support              Notes
======================= ==================== =============================
``action``              :raw-html:`&#9989;`  partially, see details below      
``actor``               :raw-html:`&#9989;`  partially, see details below      
``as``                  :raw-html:`&#10060;`       
``bool``                :raw-html:`&#9989;`          
``call``                :raw-html:`&#10060;`       
``cover``               :raw-html:`&#10060;`       
``def``                 :raw-html:`&#9989;`  only ``external``
``default``             :raw-html:`&#10060;`      
``do``                  :raw-html:`&#9989;`          
``elapsed``             :raw-html:`&#9989;`           
``emit``                :raw-html:`&#9989;`              
``enum``                :raw-html:`&#9989;`              
``event``               :raw-html:`&#9989;`              
``every``               :raw-html:`&#10060;`        
``expression``          :raw-html:`&#9989;`
``extend``              :raw-html:`&#10060;`        
``external``            :raw-html:`&#10060;`      
``fall``                :raw-html:`&#10060;`          
``float``               :raw-html:`&#9989;`    
``global``              :raw-html:`&#9989;`       
``hard``                :raw-html:`&#10060;`         
``if``                  :raw-html:`&#10060;`       
``import``              :raw-html:`&#9989;`    
``inherits``            :raw-html:`&#9989;`    
``int``                 :raw-html:`&#9989;`    
``is``                  :raw-html:`&#10060;`         
``it``                  :raw-html:`&#9989;`    
``keep``                :raw-html:`&#9989;`    
``list``                :raw-html:`&#9989;`    
``of``                  :raw-html:`&#9989;`        
``on``                  :raw-html:`&#10060;` 
``one_of``              :raw-html:`&#9989;`        
``only``                :raw-html:`&#10060;`      
``parallel``            :raw-html:`&#9989;`         
``range``               :raw-html:`&#10060;`        
``record``              :raw-html:`&#10060;`   
``remove_default``      :raw-html:`&#10060;`   
``rise``                :raw-html:`&#10060;`   
``scenario``            :raw-html:`&#9989;`     
``serial``              :raw-html:`&#9989;`     
``SI``                  :raw-html:`&#9989;`     
``string``              :raw-html:`&#9989;`     
``struct``              :raw-html:`&#9989;`   
``type``                :raw-html:`&#9989;`       
``uint``                :raw-html:`&#9989;`    
``undefined``           :raw-html:`&#10060;`     
``unit``                :raw-html:`&#9989;`       
``until``               :raw-html:`&#10060;`        
``var``                 :raw-html:`&#9989;`         
``wait``                :raw-html:`&#9989;`        
``with``                :raw-html:`&#9989;`        
======================= ==================== =============================


Composition Types
^^^^^^^^^^^^^^^^^

Composition types are ``struct``, ``actor``, ``action``, ``scenario``.

============== ==================== ===========================
Element Type   Support              Notes
============== ==================== ===========================
Event          :raw-html:`&#9989;`      
Field          :raw-html:`&#9989;`     
Constraint     :raw-html:`&#9989;`  partially
Method         :raw-html:`&#9989;`       
Coverage       :raw-html:`&#10060;`       
Modifier       :raw-html:`&#9989;`  partially (only predefined)     
============== ==================== ===========================
