# `launch_scoped_include`
Include another launch description with a limited scope. (`IncludeScopedLaunchDescription`)

This is already achievable with the standard `launch` package, but this is more convient.

This makes `IncludeScopedLaunchDescription` essentially a mix of `GroupAction` and `IncludeLaunchDescription`.

In XML the following tag can be used:
```xml
  <include_scoped file="FILE_PATH_TO_INCLUDE">
    <arg name="some_nested_arg" value="value"/>
  </include_scoped>
```
