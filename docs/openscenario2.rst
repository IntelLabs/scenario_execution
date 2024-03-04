OpenSCENARIO 2
==============

General
-------

This tool supports a subset of the `OpenSCENARIO
2 <https://www.asam.net/project-detail/asam-openscenario-v20-1/>`__ standard.

The official documentation is available
`here <https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/welcome.html>`__.

The `standard library of
OSC2 <https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/domain-model/standard_library.html>`__
was adapted to be usable by the current parsing support of scenario execution.

In the following the supported features are described.

.. role:: raw-html(raw)
   :format: html

Supported features
------------------

In the following the OpenSCENARIO 2 keywords are listed with their current support status.


======================= ==================== =============================
Element Tag             Support              Notes
======================= ==================== =============================
``action``              :raw-html:`&#9989;`  partially, see details below      
``actor``               :raw-html:`&#9989;`  partially, see details below      
``as``                  :raw-html:`&#10060;`       
``bool``                :raw-html:`&#9989;`          
``call``                :raw-html:`&#10060;`       
``cover``               :raw-html:`&#10060;`       
``def``                 :raw-html:`&#10060;`       
``default``             :raw-html:`&#10060;`      
``do``                  :raw-html:`&#9989;`          
``elapsed``             :raw-html:`&#9989;`           
``emit``                :raw-html:`&#9989;`              
``enum``                :raw-html:`&#9989;`              
``event``               :raw-html:`&#9989;`              
``every``               :raw-html:`&#10060;`        
``expression``          :raw-html:`&#10060;`    
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

============== ==================== =========
Element Type   Support              Notes
============== ==================== =========
Event          :raw-html:`&#9989;`      
Field          :raw-html:`&#9989;`     
Constraint     :raw-html:`&#9989;`  partially
Method         :raw-html:`&#10060;`       
Coverage       :raw-html:`&#10060;`       
Modifier       :raw-html:`&#10060;`       
============== ==================== =========
