C++ API
=======

The C++ API page is generated from headers/sources via Doxygen and rendered in Sphinx via Breathe.

.. ifconfig:: cpp_api_available

   Core Types
   ----------

   .. doxygenstruct:: State
      :project: dragonfly

   .. doxygenstruct:: StateDerivative
      :project: dragonfly

   .. doxygenstruct:: WingConfig
      :project: dragonfly

   .. doxygenstruct:: HarmonicSeries
      :project: dragonfly

   Core Classes
   ------------

   .. doxygenclass:: Wing
      :project: dragonfly
      :members:

   .. doxygenclass:: BladeElement
      :project: dragonfly
      :members:

   Function Index
   --------------

   .. doxygenfunction:: equationOfMotion(double, const State &, const std::vector<Wing> &, std::vector<SingleWingVectors> &)
      :project: dragonfly

.. ifconfig:: not cpp_api_available

   Doxygen was not run for this build.

   To enable C++ API extraction locally:

   .. code-block:: bash

      doxygen --version
      make -C docs html
