from typing import List

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import (
    IncludeLaunchDescription,
    PopEnvironment,
    PopLaunchConfigurations,
    PushEnvironment,
    PushLaunchConfigurations,
    ResetEnvironment,
    ResetLaunchConfigurations,
    SetLaunchConfiguration,
)
from launch.frontend import expose_action
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


@expose_action("include_scoped")
class IncludeScopedLaunchDescription(IncludeLaunchDescription):
    """A scoped version of `IncludeLaunchDescription`.

    This would be equivalent to:
    ```python
    def scoped(*args, launch_arguments, **kwargs):
        return GroupAction([
            IncludeLaunchDescription(
                *args,
                launch_arguments=launch_arguments,
                **kwargs
            )],
            scoped=True,
            forwarding=False,
            )
    ```

    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def get_sub_entities(self):
        """Get subentities."""
        ret = (
            self.launch_description_source.try_get_launch_description_without_context()
        )
        return (
            [
                PushLaunchConfigurations(),
                PushEnvironment(),
                ResetEnvironment(),
                ResetLaunchConfigurations(dict(self.launch_arguments)),
            ]
            + ([ret] if ret is not None else [])
            + [PopEnvironment(), PopLaunchConfigurations()]
        )

    def execute(self, context: LaunchContext) -> List[LaunchDescriptionEntity]:
        """Execute the action."""
        launch_description = LaunchDescription([
            PushLaunchConfigurations(),
            PushEnvironment(),
            ResetEnvironment(),
            self.launch_description_source.get_launch_description(context),
            PopEnvironment(),
            PopLaunchConfigurations(),
        ])
        # If the location does not exist, then it's likely set to '<script>' or something.
        context.extend_locals({
            "current_launch_file_path": self._get_launch_file(),
        })
        context.extend_locals({
            "current_launch_file_directory": self._get_launch_file_directory(),
        })

        # Do best effort checking to see if non-optional, non-default declared arguments
        # are being satisfied.
        my_argument_names = [
            perform_substitutions(context, normalize_to_list_of_substitutions(arg_name))
            for arg_name, arg_value in self.launch_arguments
        ]
        declared_launch_arguments = launch_description.get_launch_arguments_with_include_launch_description_actions()
        for argument, ild_actions in declared_launch_arguments:
            if argument._conditionally_included or argument.default_value is not None:
                continue
            argument_names = my_argument_names
            if ild_actions is not None:
                for ild_action in ild_actions:
                    argument_names.extend(
                        ild_action._try_get_arguments_names_without_context()
                    )
            if argument.name not in argument_names:
                raise RuntimeError(
                    "Included launch description missing required argument '{}' "
                    "(description: '{}'), given: [{}]".format(
                        argument.name, argument.description, ", ".join(argument_names)
                    )
                )

        # Create actions to set the launch arguments into the launch configurations.
        set_launch_configuration_actions = []
        for name, value in self.launch_arguments:
            set_launch_configuration_actions.append(SetLaunchConfiguration(name, value))

        # Set launch arguments as launch configurations and then include the launch description.
        return [*set_launch_configuration_actions, launch_description]
