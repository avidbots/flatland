.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Yaml Preprocessor
==============================

Flatland Server has a lua preprocessor for YAML with simple bindings for the environment variables and rosparam.
The intent is to be able to build parametric models that are defined by dimensions and flags found in either environment variables or rosparam, in a similar way to xacro+urdf. Because this is parsed at model load time, any roslaunch rosparam loading will have completed and those parameters will be available.

body/joint/plugin enabled flag
------------------------------
Model bodies, plugins and joints now have a new flag `enabled` which can be set to true or false either directly in the yaml, or based on more complex logic from a lua `$eval` string that returns "true" or "false". Disabled bodies, plugins and joints are skipped during yaml loading, and as a result are never instantiated. From Flatland's perspective `enabled: false` causes the affected body/plugin/joint to be deleted. 

bindings for env and param
-------------------------------

Additional lua function bindings (beyond the normal standard libraries such as string, math, etc.):

.. code-block:: lua

  -- returns an environment variable, blank string + warning if not found
  env(EnvName)
  -- returns an environment variable, Default if not found
  env(EnvName, Default)
  -- returns a rosparam, blank string + warning if not found
  param(ParamPath)
  -- returns a rosparam, Default if not found
  param(ParamPath, Default)

Sample expressions
------------------------------

.. code-block:: yaml

  foo: $eval "Some arbitrary LUA expression"
  bar: |  # Multiline string
    $eval  -- $eval flag required to trigger LUA parsing
    if env("FRONT_WHEEL",true) then
      return 1.2
    else
      return 2.4
    end

Lua expressions can explicitly `return` their value, but if no `return` is given, one will be prepended to the statement.

env + param examples
-----------------------------

.. code-block:: yaml

  # in: (SOME_ENV not set)
  foo: $eval env("SOME_ENV")
  # out:
  foo: ""

.. code-block:: yaml

  # in: (SOME_ENV not set)
  foo: $eval env("SOME_ENV", false)
  # out:
  foo: false

.. code-block:: yaml

  # in: (export SOME_ENV=true)
  foo: $eval env("SOME_ENV")
  # out:
  foo: true

.. code-block:: yaml

  # in: (rosparam /test/param not set)
  foo: $eval param("/test/param", 0)/2.0
  # out:
  foo: 0

.. code-block:: yaml

  # in: (rosparam /test/param set to 5.0)
  foo: $eval param("/test/param", 0)/2.0 + 1
  # out:
  foo: 2.5

