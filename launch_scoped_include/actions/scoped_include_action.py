from pathlib import Path
from typing import List, Union

import launch.logging
from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
    LaunchDescriptionSource,
    SomeSubstitutionsType,
)
from launch.action import Action
from launch.actions import (
    PopEnvironment,
    PopLaunchConfigurations,
    PushEnvironment,
    PushLaunchConfigurations,
    ResetEnvironment,
    ResetLaunchConfigurations,
    SetLaunchConfiguration,
    IncludeLaunchDescription,
)
from launch.frontend import Entity, Parser, expose_action
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


@expose_action("include_scoped")
class IncludeScopedLaunchDescription(IncludeLaunchDescription):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def get_sub_entities(self):
        """Get subentities."""
        ret = (
            self.launch_description_source.try_get_launch_description_without_context()
        )
        # configuration_sets = [
        #         SetLaunchConfiguration(k, v) for k, v in self.launch_arguments
        #     ]
        print(self.launch_arguments)
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
            # ResetLaunchConfigurations(self.launch_arguments),
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


# @expose_action("include_scoped")
# class IncludeScopedLaunchDescription(Action):
#     def __init__(
#         self,
#         launch_description_source: Union[
#             LaunchDescriptionSource, SomeSubstitutionsType
#         ],
#         **kwargs,
#     ) -> None:
#         super().__init__(**kwargs)
#         if not isinstance(launch_description_source, LaunchDescriptionSource):
#             launch_description_source = AnyLaunchDescriptionSource(
#                 launch_description_source
#             )
#         self.__launch_description_source = launch_description_source
#         self.__logger = launch.logging.get_logger(__name__)

#     @classmethod
#     def parse(cls, entity: Entity, parser: Parser):
#         """Return `IncludeScopedLaunchDescription` action and kwargs for constructing it."""
#         _, kwargs = super().parse(entity, parser)
#         file_path = parser.parse_substitution(entity.get_attr("file"))
#         kwargs["launch_description_source"] = file_path
#         # args = entity.get_attr('arg', data_type=List[Entity], optional=True)
#         # if args is not None:
#         #     kwargs['launch_arguments'] = [
#         #         (
#         #             parser.parse_substitution(e.get_attr('name')),
#         #             parser.parse_substitution(e.get_attr('value'))
#         #         )
#         #         for e in args
#         #     ]
#         #     for e in args:
#         #         e.assert_entity_completely_parsed()
#         return cls, kwargs

#     @property
#     def launch_description_source(self) -> LaunchDescriptionSource:
#         """Getter for self.__launch_description_source."""
#         return self.__launch_description_source

#     def _get_launch_file(self):
#         return Path(self.__launch_description_source.location).absolute()

#     def _get_launch_file_directory(self):
#         launch_file_location = self._get_launch_file()
#         if launch_file_location.exists():
#             launch_file_location = launch_file_location.parent
#         else:
#             # If the location does not exist, then it's likely set to '<script>' or something
#             # so just pass it along.
#             launch_file_location = self.__launch_description_source.location
#         return launch_file_location

#     def get_sub_entities(self):
#         """Get subentities."""
#         ret = self.__launch_description_source.try_get_launch_description_without_context()
#         return (
#             [PushLaunchConfigurations(), PushEnvironment()]
#             + ([ret] if ret is not None else [])
#             + [PopEnvironment(), PopLaunchConfigurations()]
#         )

#     def _try_get_arguments_names_without_context(self):
#         try:
#             context = LaunchContext()
#             return [
#                 # perform_substitutions(
#                 #     context, normalize_to_list_of_substitutions(arg_name)
#                 # )
#                 # for arg_name, arg_value in self.__launch_arguments
#             ]
#         except Exception as exc:
#             self.__logger.debug(
#                 "Failed to get launch arguments names for launch description "
#                 f"'{self.__launch_description_source.location}', "
#                 f"with exception: {str(exc)}"
#             )
#         return None

#     def execute(self, context: LaunchContext) -> List[LaunchDescriptionEntity]:
#         """Execute the action."""
#         launch_description = self.__launch_description_source.get_launch_description(
#             context
#         )
#         # If the location does not exist, then it's likely set to '<script>' or something.
#         context.extend_locals({
#             "current_launch_file_path": self._get_launch_file(),
#         })
#         context.extend_locals({
#             "current_launch_file_directory": self._get_launch_file_directory(),
#         })

#         # Do best effort checking to see if non-optional, non-default declared arguments
#         # are being satisfied.
#         my_argument_names = [
#             # perform_substitutions(context, normalize_to_list_of_substitutions(arg_name))
#             # for arg_name, arg_value in self.launch_arguments
#         ]
#         declared_launch_arguments = launch_description.get_launch_arguments_with_include_launch_description_actions()
#         for argument, ild_actions in declared_launch_arguments:
#             if argument._conditionally_included or argument.default_value is not None:
#                 continue
#             argument_names = my_argument_names
#             if ild_actions is not None:
#                 for ild_action in ild_actions:
#                     argument_names.extend(
#                         ild_action._try_get_arguments_names_without_context()
#                     )
#             if argument.name not in argument_names:
#                 raise RuntimeError(
#                     "Included launch description missing required argument '{}' "
#                     "(description: '{}'), given: [{}]".format(
#                         argument.name, argument.description, ", ".join(argument_names)
#                     )
#                 )

#         # Create actions to set the launch arguments into the launch configurations.
#         set_launch_configuration_actions = []
#         # for name, value in self.launch_arguments:
#             # set_launch_configuration_actions.append(SetLaunchConfiguration(name, value))

#         # Set launch arguments as launch configurations and then include the launch description.
#         return [*set_launch_configuration_actions, launch_description]
