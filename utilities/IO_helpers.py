# type: ignore
import logging
from collections.abc import Collection, Iterable
from dataclasses import dataclass, field
from enum import Enum, EnumType
from inspect import _empty, signature
from itertools import zip_longest
from types import EllipsisType
from typing import Any, Callable, Optional, get_args, get_origin

import ntcore
import wpimath.units as units
from phoenix6.base_status_signal import BaseStatusSignal

# TODO: add CANdle after that's added to phoenix6
from phoenix6.hardware import CANcoder, CANrange, ParentDevice, Pigeon2, TalonFX
from phoenix6.status_signal import StatusSignal
from wpilib import RobotBase, Timer

from utilities.elasticlib import Notification, NotificationLevel, NotificationManager

POSSIBLE_TOPICS = [int, float, str, bool, bytes]  # Doesn't include WPIstructs

# Don't change these
NT_INSTANCE = ntcore.NetworkTableInstance.getDefault()
MAIN_TABLE = "IO"

OPTIONS = ntcore.PubSubOptions(sendAll=True)

logger = logging.getLogger("IO")

number_publishers_produced = 0

type AnyAmountStr = str | Iterable[str, ...]


@dataclass
class PublisherInfo:
    data: Callable[[], Any] | tuple[Callable[[], Any], Callable[[Any], Any]] | str
    last_val: Optional[Any]
    setter: Callable[[Any], None]
    modifier: Callable[[Any], Any] | None
    condition: Callable[[], bool] | None
    cls: object

    def __post_init__(self):
        global number_publishers_produced
        number_publishers_produced += 1


@dataclass
class PublisherSpecifications:
    groups: dict[AnyAmountStr, str] = field(default_factory=dict)
    modifiers: dict[AnyAmountStr, Callable[[Any], Any]] = field(default_factory=dict)
    conditions: dict[AnyAmountStr, Callable[[], bool]] = field(default_factory=dict)
    container: str | None = None
    ignore_fields: set[str] = field(default_factory=set)
    high_resolution: set[str] = field(default_factory=set)
    rename: bool = True
    insert_class_name: set[str] = field(default_factory=set)


def auto_log(cls):
    """Logs all possible fields in a class.

    Provides a way to easily log tons of stuff without clogging up the main component
    classes. All info will be logged in the parent `IO` folder then individual subfolders
    deppending on how you partition it.

    There are a couple static fields you can define to add extra functionality:
        - GROUPS (dict): this should be a dict of prefix name to folder name. All variables/functions with the prefix
            will be added to the defined folder
        - Container (str): this will create this IO's folder within another folder. This is primarily for IO classes
            that you need to create multiple of since it will group them neatly.
        - IGNORE_FIELDS (set): put the string names of any fields that you do not want to log in this set. This allows for you
            to put more complicated objects or miscellaneous within the IO without having to worry about causing any errors.
        - RENAME_FIELDS (bool): This defaults to True and will rename all fields and functions added to the IO for better
            display. Currently will remove any leading `_`, `get_`, or `is_` and replace other `_` with spaces.
        - MODIFIER (dict): A single variable name or list of variable names paired with a function. The function will be called
            on the variable when logged, this is primarily useful for unit conversion or scaling
        - CONDITIONS (dict): A single variable name or list of variable names paired with a function. The data will only be logged
            if the condition is True

    All of these fields can also be put into a single `PublisherSpecifications` object using the SPECIFICATIONS static variable.
    """
    orig_init = cls.__init__

    def new_init(self, *args, **kwargs):
        orig_init(self, *args, **kwargs)
        specifications = _retrieve_specifications(self)

        IO._add_list_to_rotating_publishers(
            _create_publishers_from_class(self, specifications)
        )
        IO._add_list_to_rotating_publishers(
            _create_publishers_for_static_annotations(cls, specifications, self)
        )

    cls.__init__ = new_init

    # This is pretty pointless. But if someone decides to create this as a magic component, it won't
    # immediately crash code
    cls.execute = lambda self: None

    return cls


class IO:
    _rotating_publisher_list: list[list[PublisherInfo]] = [[]]

    _status_signal_info_list: list[tuple[str, object]] = []
    _status_signal_refresh_list: list[StatusSignal] = []
    _status_signal_refresh_flag: bool = False

    _fallback_list = []
    _high_resolution_publishers = []

    _adding_index = 0  # Index for adding new publishers
    _selection_index = 0  # Index for sending publishers

    MAX_LIST_SIZE = 20

    # Add to this to track more things for each type
    DEVICE_CALLABLES: dict[ParentDevice, tuple[Callable[[], StatusSignal], ...]] = {
        TalonFX: (
            TalonFX.get_supply_voltage,
            TalonFX.get_motor_voltage,
            TalonFX.get_supply_current,
            TalonFX.get_stator_current,
            TalonFX.get_torque_current,
            TalonFX.get_motor_stall_current,
            TalonFX.get_device_temp,
            TalonFX.get_ancillary_device_temp,
            TalonFX.get_fault_field,
            TalonFX.get_sticky_fault_field,
        ),
        CANcoder: (
            CANcoder.get_supply_voltage,
            CANcoder.get_magnet_health,
            CANcoder.get_fault_field,
            CANcoder.get_sticky_fault_field,
        ),
        CANrange: (
            CANrange.get_supply_voltage,
            CANrange.get_ambient_signal,
            CANrange.get_fault_field,
            CANcoder.get_sticky_fault_field,
        ),
        Pigeon2: (
            Pigeon2.get_supply_voltage,
            Pigeon2.get_temperature,
            Pigeon2.get_fault_field,
            Pigeon2.get_sticky_fault_field,
        ),
    }

    def __init__(self, name: Optional[str] = None) -> None:
        """Base class for all IOs.

        You don't have to subclass this, but doing so provides some nice helper functions to
        optimize logging.

        Params:
            name (Optional): the name of the folder where IO data is published. You usually want to define
            this if you are creating multiple of one IO class (eg. swerve modules), defaults to the name of the IO class.
        """
        if name:
            # HACK: Can't create another field otherwise without it also being logged, so this is the
            # best alternative to ensure names get passed through.
            self.__class__.__name__ = name

    @staticmethod
    def update_publishers(max_timespan: units.milliseconds = 4) -> None:
        """Updates the value of all publishers to NetworkTables.

        This will update everything added to each IO, providing a centralized interface for
        updating values. This method sends publisher data in batches, staggering each batch
        by the 20ms control loop. This ensures that no batch takes up too much of the loop time,
        improving loop performance at the cost of a slightly slower send rate.
        """
        if not IO._rotating_publisher_list or not IO._rotating_publisher_list[0]:
            logger.warning(
                "Rotating publishers list is empty, nothing is being logged."
            )
            return

        start = Timer.getFPGATimestamp()
        IO._handle_signal_refreshing()

        for publisher in IO._high_resolution_publishers:
            IO._send_to_network_tables(publisher)

        for publisher in IO._fallback_list:
            IO._send_to_network_tables(publisher)
        IO._fallback_list.clear()

        for i, publisher in enumerate(IO._rotating_publisher_list[IO._selection_index]):
            if Timer.getFPGATimestamp() - start > max_timespan / 1000:
                remaining_publishers = IO._rotating_publisher_list[IO._selection_index][
                    i:
                ]

                # logger.info(
                #     "Overran allocated loop time of %s, transfered %s publishers into fallback list\nCurrent Timestamp: %s",
                #     max_timespan,
                #     len(remaining_publishers),
                #     Timer.getFPGATimestamp(),
                # )
                IO._fallback_list.extend(remaining_publishers)
                break

            IO._send_to_network_tables(publisher)

        IO._selection_index = (IO._selection_index + 1) % len(
            IO._rotating_publisher_list
        )

    @staticmethod
    def flush_publishers() -> None:
        """Flushes all publishers to NetworkTables.

        This will avoid staggering sending data and send everything all at once. Only
        call this during initialization because otherwise it can take a large chunk of the
        20ms control loop to execute.
        """
        IO._handle_signal_refreshing()

        if not IO._rotating_publisher_list or not IO._rotating_publisher_list[0]:
            logger.info("Rotating publishers list is empty, nothing will be flushed.")
            return

        start = Timer.getFPGATimestamp()
        for publisher_list in IO._rotating_publisher_list:
            for publisher in publisher_list:
                IO._send_to_network_tables(publisher, True)

        logger.info(
            f"Flushed all publishers in {Timer.getFPGATimestamp() - start:.5f}s"
        )

    def add_function(
        self,
        func: Callable[[], Any],
        modifier: Optional[Callable[[Any], Any]] = None,
        extra: Optional[str] = None,
    ) -> None:
        """Adds a function to be logged by the IO.

        This is useful for anything that needs to be logged in disabled, since most fields won't be updated during that period.
        The logged function will be called every periodic cycle, so any extra fields you add in there will also be updated 24/7
        so you can mix the two forms of logging and control what is updated, when.

        Args:
            func: The function Callable to be logged (the name of the function without parenthesis)
            modifier (Optional): This will be called on top of the provided function. Acts as a quick way to convert units and modulus
                values without having to define another function.
            extra (Optional): The extra folder in NetworkTables where this function will be logged to. This usually should not be used,
                instead you can modify the GROUPS static field in the IO to group functions along with fields
        """
        specifications = _retrieve_specifications(self)
        name = func.__name__
        clean_name = _filter_name(func.__name__)

        io_extra = _get_dict_value_from_specifications(
            name, clean_name, specifications_dict=specifications.groups
        )

        io_modifier = _get_dict_value_from_specifications(
            name, clean_name, specifications_dict=specifications.modifiers
        )

        publisher = _create_publisher_from_function(
            func,
            self,
            extra_table=extra or io_extra,
            modifier=modifier or io_modifier,
            specifications=specifications,
        )

        if not publisher:
            return

        if any(
            check_name((name, clean_name), provided=high_res_field)
            for high_res_field in specifications.high_resolution
        ):
            IO._high_resolution_publishers.append(publisher)
            return

        IO._add_value_to_rotating_publishers(publisher)

    def add_ctre_device(
        self, *devices: ParentDevice, names: Optional[str | tuple[str, ...]] = None
    ) -> None:
        """Adds a CTRE device to the IO.

        This is useful for checking on the health and status of certain CTRE devices. Currently logs
        some important health info like temperature, supply current draw, fault counts, etc.

        The method will log status info from the following devices:
            - CANcoder
            - CANdle (TODO -> verify implementation)
            - CANrange
            - TalonFX
            - Pigeon2

        Args:
            devices: the device(s) to log on this IO
            names (Optional): custom names to add to the devices on network tables. If not provided it
                will default to device model + device ID + device CAN network.
        """

        if names is not None and not isinstance(names, tuple):
            # Using a tuple here would keep names as a string, so need to use a list
            names = [names]

        for device in devices:
            if type(device) not in IO.DEVICE_CALLABLES:
                raise ValueError(
                    f"This method cannot log values from object {type(device)}"
                )

        # So names need not be entered for every device
        for device, name in zip_longest(devices, names if names else ()):
            if name is None:
                name = (
                    device._device_identifier.model.upper()
                    + " "
                    + str(device.device_id)
                    + device.network
                )
                NotificationManager.add_conditional_notification(
                    Notification(
                        level=NotificationLevel.ERROR,
                        title=f"{device._device_identifier.model.upper()} is not connected",
                        description=f"Check {device._device_identifier.model.upper()} with id {device.device_id} on bus {device.network}",
                    ),
                    lambda: not (device.is_connected or RobotBase.isSimulation()),  # noqa: B023
                )

            for static in IO.DEVICE_CALLABLES.get(type(device), []):
                try:
                    instance_func = getattr(device, static.__name__)
                except AttributeError:
                    logger.info(
                        f"Could not find function '{static.__name__}' on device '{device._device_identifier.model}'"
                    )
                    continue
                self.add_function(
                    instance_func,
                    extra=name,
                )

    @staticmethod
    def get_number_publishers():
        global number_publishers_produced
        return number_publishers_produced

    @staticmethod
    def _create_refresh_list():
        IO._status_signal_refresh_list.clear()

        for info in IO._status_signal_info_list:
            name, obj = info

            signal = getattr(obj, name, None)
            if signal is not None and isinstance(signal, StatusSignal):
                IO._status_signal_refresh_list.append(signal)

        if len(IO._status_signal_refresh_list) == len(IO._status_signal_info_list):
            IO._status_signal_refresh_flag = True  # Every status

    @staticmethod
    def _handle_signal_refreshing():
        if not IO._status_signal_refresh_list or not IO._status_signal_refresh_flag:
            IO._create_refresh_list()  # Protects against potential late binding

        try:
            BaseStatusSignal.refresh_all(IO._status_signal_refresh_list)
        except Exception as e:
            logger.warning("Could not refresh status signals\nStack trace: %s", e)

    @staticmethod
    def _add_list_to_rotating_publishers(publishers: list[PublisherInfo]) -> None:
        """Adds a list to the rotating publishers list. If the list overflows the max capacity of the
           rotating publishers list, then it will be broken up into smaller batches.

        Args:
            publishers (list): list of PublisherInfo to be added
        """
        current_list = IO._rotating_publisher_list[IO._adding_index]
        if len(publishers) + len(current_list) <= IO.MAX_LIST_SIZE:
            current_list.extend(publishers)
            return

        remaining_publishers = publishers[:]

        while remaining_publishers:
            current_batch = IO._rotating_publisher_list[IO._adding_index]
            space_left_in_current_batch = IO.MAX_LIST_SIZE - len(current_batch)

            if space_left_in_current_batch == 0:
                IO._adding_index += 1
                if IO._adding_index >= len(IO._rotating_publisher_list):
                    IO._rotating_publisher_list.append([])
                continue

            num_to_add = min(len(remaining_publishers), space_left_in_current_batch)
            current_batch.extend(remaining_publishers[:num_to_add])

            remaining_publishers = remaining_publishers[num_to_add:]

            if remaining_publishers:
                IO._adding_index += 1
                if IO._adding_index >= len(IO._rotating_publisher_list):
                    IO._rotating_publisher_list.append([])

    @staticmethod
    def _add_value_to_rotating_publishers(publisher: PublisherInfo) -> None:
        """Adds a single value to the rotating publishers list. If adding it will exceed the
           maximum capacity of the rotating list, then it will be appended into a new inner list.

        Args:
            publisher: the PublisherInfo to be added
        """
        current_list = IO._rotating_publisher_list[IO._adding_index]
        if len(current_list) < IO.MAX_LIST_SIZE:
            current_list.append(publisher)
        else:
            IO._adding_index += 1
            if IO._adding_index >= len(IO._rotating_publisher_list):
                IO._rotating_publisher_list.append([])
                IO._rotating_publisher_list[IO._adding_index].append(publisher)

    @staticmethod
    def _send_to_network_tables(publisher: PublisherInfo, flush: bool = False) -> None:
        """Sends publisher info to NetworkTables.

        This will call any modifiers/functions and get all instance fields of IO classes.
        This will skip over any publisher which raises an error, logging the results

        Args:
            publisher: The publisher info to be logged in NetworkTables
            flush: if set to True, this will avoid all duplicate checks and just send
                the data
        """
        if publisher.condition is not None and not publisher.condition():
            return

        target = None
        data = publisher.data
        obj = publisher.cls

        if callable(data):
            target = data()
        else:
            try:
                target = getattr(obj, data, None)
            except AttributeError:
                logger.error(
                    f"Attribute '{data}' not found on instance '{type(obj).__name__}'. Skipping publisher."
                )
                return

        if target is None:
            return  # Provides any late binding (eg. static attributes) from crashing stuff

        if isinstance(target, StatusSignal):
            target = (
                target.value
                if not isinstance(target.value, Enum)
                else target.value.name
            )

        if publisher.modifier:
            try:
                target = publisher.modifier(target)
            except (TypeError, ValueError, AttributeError) as e:
                logger.error(
                    f"Error processing publisher '{data}':{e} on instance '{type(obj).__name__}'. Skipping over it."
                )
                return
            except Exception as e:
                logger.error(
                    f"An unexpected error occurred for publisher '{data}': {e} on instance '{type(obj).__name__}'. Review its source."
                )
                return

        if target == publisher.last_val and not flush:
            return

        publisher.setter(target)
        publisher.last_val = target


def _retrieve_specifications(obj: object) -> PublisherSpecifications:
    new_specifications_flag = False
    specifications = getattr(obj, "SPECIFICATIONS", None)

    if specifications is None:
        specifications = PublisherSpecifications()
        new_specifications_flag = True

    for name, value in vars(specifications).items():
        static_field_value = getattr(obj, name.upper(), None)

        if static_field_value is None:
            continue

        if not value or (static_field_value != value and new_specifications_flag):
            setattr(specifications, name, static_field_value)

    return specifications


def _get_topic_callable(
    table: ntcore.NetworkTable, desired_type: type, arr: bool
) -> Callable[[], ntcore.Topic]:
    """
    Returns the appropriate NetworkTable topic method based on the given type and array status.

    Args:
        table (ntcore.NetworkTable): The NetworkTable to get the topic from.
        desired_type (Type): The data type to publish (e.g., int, float, str).
        arr (bool): Whether the data is an array.

    Returns:
        Callable[[], ntcore.Topic]: A callable that returns a topic for the given type.
    """
    topic_types: dict[type, Callable[[], ntcore.Topic]] = {
        int: table.getIntegerTopic,
        float: table.getFloatTopic,
        str: table.getStringTopic,
        bool: table.getBooleanTopic,
        bytes: table.getRawTopic,
    }

    array_types: dict[type, Callable[[], ntcore.Topic]] = {
        int: table.getIntegerArrayTopic,
        float: table.getDoubleArrayTopic,
        str: table.getStringArrayTopic,
        bool: table.getBooleanArrayTopic,
    }

    return topic_types.get(desired_type) if not arr else array_types.get(desired_type)


def _filter_name(name: str) -> str:
    """
    Cleans and formats a variable or function name for use as a topic name.

    Args:
        name (str): The original name.

    Returns:
        str: The cleaned name with prefixes removed and underscores replaced with spaces.
    """
    name = name.removeprefix("_")
    name = name.removeprefix("get_")
    name = name.removeprefix("is_")
    name = name.replace("_", " ")

    return name


def _get_type_from_value(val: Any) -> tuple[type, bool]:
    return_type = None

    if isinstance(val, Collection) and not isinstance(val, (str, bytes)):
        if isinstance(val, tuple) and not (
            len({type(_) for _ in val}) == 1
        ):  # No need for ellipsis type checks here
            raise ValueError("Non homogenous tuple provided")

        return_type = type(val[0])
        arr = True
    else:
        return_type = type(val)
        arr = False

    return (return_type, arr)


def _get_type_from_annotation(annotation: type) -> tuple[type, bool]:
    def check_invalid_tuple(
        annotation: type,
    ):  # Makes sure tuple is homogenous or tuple[T, ...]
        if annotation is not tuple:
            return True

        return (
            len({get_args(annotation)}) == 1 or get_args(annotation)[-1] is EllipsisType
        )

    origin = get_origin(annotation)  # Returns outside type eg. list[int] = list

    return_type = annotation
    arr = False

    if origin is not None:
        if origin is StatusSignal:
            return_type = get_args(annotation)[0]  # Inside type
        elif issubclass(origin, Collection):
            if not check_invalid_tuple(origin):
                raise ValueError("Non-homogenous tuple provided")

            return_type = get_args(annotation)[0]
            arr = True

    return (return_type, arr)


def _register_wildcards(name: str, provided: str) -> bool:
    if provided.endswith("*") and provided.startswith("*"):
        return provided.replace("*", "") in name
    elif provided.endswith("*"):
        return name.startswith(provided.removesuffix("*"))
    elif provided.startswith("*"):
        return name.endswith(provided.removeprefix("*"))
    return False


def check_name(names: str | Iterable[str], provided: str) -> bool:
    if isinstance(names, str):
        names = [names]

    return any(
        name == provided or _register_wildcards(name, provided) for name in names
    )


def _get_dict_value_from_specifications(
    *names: str, specifications_dict: dict[str, Any]
) -> Any:
    if not specifications_dict:
        return None

    for keys, value in specifications_dict.items():
        if isinstance(keys, str):
            keys = [keys]

        if any(check_name(names, key) for key in keys):
            return value

    return None


def _create_publisher_for_type(
    val: Any, nt_instance: ntcore.NetworkTable, topic_name: str
) -> ntcore.Publisher:
    """
    Creates a NetworkTables publisher for a given type or instance.

    Args:
        val (Any): The value or type to determine the publisher type.
        nt_instance (ntcore.NetworkTable): The NetworkTable to publish to.
        topic_name (str): The name of the topic.

    Returns:
        ntcore.Publisher: A publisher object for the given type.

    Raises:
        ValueError: If the type is not supported for publishing.
    """
    publisher = None

    # Generic types such as list[T] don't count as types
    # so second condition makes up for that
    publisher_type, arr = (
        _get_type_from_value(val)
        if not (isinstance(val, type) or get_origin(val))
        else _get_type_from_annotation(val)
    )

    if isinstance(publisher_type, (Enum, EnumType)):
        publisher_type = str  # Probably want the name of the enum instead of type

    if hasattr(publisher_type, "WPIStruct"):
        if arr:
            publisher = nt_instance.getStructArrayTopic(
                topic_name, publisher_type
            ).publish()
        else:
            publisher = nt_instance.getStructTopic(topic_name, publisher_type).publish()
    elif publisher_type in POSSIBLE_TOPICS:
        topic_method = _get_topic_callable(nt_instance, publisher_type, arr)
        publisher = topic_method(topic_name).publish()

    if publisher is not None:
        return publisher
    else:
        raise ValueError(f"Cannot create publisher for type {publisher_type!r}")


def _create_publishers_for_static_annotations(
    cls: type, specifications: PublisherSpecifications, obj: object
) -> list[PublisherInfo]:
    publishers = []
    full_path = f"{MAIN_TABLE}{'/' + specifications.container if specifications.container is not None else ''}{'/' + cls.__name__}"
    nt_instance = NT_INSTANCE.getTable(full_path)

    for name, annotation in cls.__annotations__.items():
        if get_origin(annotation) is StatusSignal:
            IO._status_signal_info_list.append((name, obj))

        if name in specifications.ignore_fields:
            continue

        clean_name = _filter_name(name) if specifications.rename else name
        modifier = _get_dict_value_from_specifications(
            clean_name, name, specifications_dict=specifications.modifiers
        )  # No real way to check types here

        if (
            name in specifications.insert_class_name
            or clean_name in specifications.insert_class_name
        ):
            clean_name = cls.__name__.removesuffix("IO") + " " + clean_name

        if any(
            check_name((name, clean_name), provided=field)
            for field in specifications.ignore_fields
        ):
            logger.info(
                f"Field {name} in {cls.__name__} is ignored. Skipping over logging it."
            )
            continue

        condition = _get_dict_value_from_specifications(
            clean_name, name, specifications_dict=specifications.conditions
        )

        extra_instance = _get_dict_value_from_specifications(
            clean_name, name, specifications_dict=specifications.groups
        )
        if extra_instance is not None:
            extra_instance = nt_instance.getSubTable(extra_instance)

        publisher = _create_publisher_for_type(
            annotation, extra_instance or nt_instance, clean_name
        )

        info = PublisherInfo(name, None, publisher.set, modifier, condition, obj)

        if any(
            check_name((name, clean_name), provided=high_res_field)
            for high_res_field in specifications.high_resolution
        ):
            IO._high_resolution_publishers.append(info)
            continue

        publishers.append(info)

    return publishers


def _create_publishers_from_class(
    cls: object, specifications: PublisherSpecifications
) -> list[PublisherInfo]:
    """
    Creates publishers for each field in a class.

    Args:
        cls (object): The class instance to inspect for fields.
        groups (Optional[dict[str, str]]): Optional mapping of field prefixes to subtable names.
        container (Optional[str]): Optional container name for topic path.
        ignore_fields (Optional[set[str]]): Fields to skip when creating publishers.
        rename (bool): Whether to rename fields using `_filter_name`.

    Returns:
        list[PublisherInfo]: A list of publisher metadata entries.
    """
    publishers = []

    full_path = f"{MAIN_TABLE}{'/' + specifications.container if specifications.container is not None else ''}{'/' + type(cls).__name__}"
    nt_instance = NT_INSTANCE.getTable(full_path)

    for name, val in vars(cls).items():
        if isinstance(val, StatusSignal):
            IO._status_signal_info_list.append((name, cls))

        clean_name = _filter_name(name) if specifications.rename else name

        if any(
            check_name((name, clean_name), provided=field)
            for field in specifications.ignore_fields
        ):
            logger.info(
                f"Field {name} in {type(cls).__name__} is ignored. Skipping over logging it."
            )
            continue

        extra_instance = _get_dict_value_from_specifications(
            name, clean_name, specifications_dict=specifications.groups
        )

        if extra_instance is not None:
            extra_instance = nt_instance.getSubTable(extra_instance)

        modifier = _get_dict_value_from_specifications(
            name, clean_name, specifications_dict=specifications.modifiers
        )
        condition = _get_dict_value_from_specifications(
            name, clean_name, specifications_dict=specifications.conditions
        )

        if modifier:
            val = modifier(val)

        if (
            name in specifications.insert_class_name
            or clean_name in specifications.insert_class_name
        ):
            clean_name = type(cls).__name__.removesuffix("IO") + " " + clean_name

        publisher = _create_publisher_for_type(
            val, extra_instance or nt_instance, clean_name
        )

        info = PublisherInfo(name, None, publisher.set, modifier, condition, cls)

        if any(
            check_name((name, clean_name), provided=high_res_field)
            for high_res_field in specifications.high_resolution
        ):
            IO._high_resolution_publishers.append(info)
            continue

        publishers.append(info)

    return publishers


def _create_publisher_from_function(
    func: Callable[[], Any],
    cls: object,
    specifications: PublisherSpecifications,
    extra_table: Optional[str] = None,
    modifier: Optional[Callable[[], Any]] = None,
) -> PublisherInfo:
    """
    Creates a publisher from a function's return value.

    Args:
        func (Callable[[], Any]): A zero-argument method (usually a getter).
        cls (object): The class instance the method belongs to.
        extra_table (Optional[str]): An optional subtable to organize the topic under.
        container (Optional[str]): Optional container name for topic path.
        modifier (Callable[[], Any]): Optional function to modify the function's return type.
        rename (bool): Whether to rename the function using `_filter_name`.

    Returns:
        PublisherInfo: Metadata about the created publisher.

    Raises:
        SyntaxError: If the function does not follow the expected format.
        ValueError: If the topic name collides with a folder name.
    """
    type_hints = signature(func)
    if type_hints.return_annotation == _empty:
        raise SyntaxError("function must have a type-hinted return type")
    elif len(type_hints.parameters) > 1:
        raise SyntaxError("logged function cannot have any parameters other than self")

    rtype = type_hints.return_annotation

    if modifier is not None:
        # There's probably a better way. But some built-in functions have weird typehints
        rtype = type(modifier(func()))

    full_path = f"{MAIN_TABLE}{'/' + specifications.container if specifications.container is not None else ''}{'/' + type(cls).__name__}{'/' + extra_table if extra_table is not None else ''}"
    nt_instance = NT_INSTANCE.getTable(full_path)

    clean_name = _filter_name(func.__name__) if specifications.rename else func.__name__

    if clean_name == nt_instance.getPath().split("/")[-1]:
        raise ValueError(
            "Function topic has same name as subfolder. Please change one otherwise there can be logging issues."
        )

    if any(
        check_name((func.__name__, clean_name), provided=field)
        for field in specifications.ignore_fields
    ):
        logger.info(
            f"Field {func.__name__} in {type(cls).__name__} is ignored. Skipping over logging it."
        )
        return

    if (
        clean_name in specifications.insert_class_name
        or func.__name__ in specifications.insert_class_name
    ):
        clean_name = type(cls).__name__.removesuffix("IO") + " " + clean_name

    publisher = _create_publisher_for_type(rtype, nt_instance, clean_name)
    return PublisherInfo(func, None, publisher.set, modifier, None, cls)
