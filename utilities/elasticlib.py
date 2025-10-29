# pylint: disable = all

"""
Most of this file is taken directly from https://github.com/Gold872/elastic-dashboard and allows for
easy integration between code and the elastic dashboard. The documentation for elastic
can be found here https://frc-elastic.gitbook.io/docs

NotificationManager is a custom class to help with sending conditional notifications and minimizing overlap
"""

import contextlib
import json
import logging
from collections import deque
from enum import Enum
from typing import Callable, Optional

import wpimath.units as units
from ntcore import NetworkTableInstance, PubSubOptions
from wpilib import Timer


class NotificationLevel(Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


class Notification:
    """Represents an notification with various display properties."""

    def __init__(
        self,
        level: NotificationLevel = NotificationLevel.INFO,
        title: str = "",
        description: str = "",
        display_time: int = 3000,
        width: float = 350,
        height: float = -1,
    ) -> None:
        """
        Initializes an ElasticNotification object.

        Args:
            level (str): The severity level of the notification. Default is 'INFO'.
            title (str): The title of the notification. Default is an empty string.
            description (str): The description of the notification. Default is an empty string.
            display_time (int): Time in milliseconds for which the notification should be displayed. Default is 3000 ms.
            width (float): Width of the notification display area. Default is 350.
            height (float): Height of the notification display area. Default is -1 (automatic height).
        """
        self.level = level
        self.title = title
        self.description = description
        self.display_time = display_time
        self.width = width
        self.height = height


__selected_tab_topic = None
__selected_tab_publisher = None

__notification_topic = None
__notification_publisher = None


def send_notification(notification: Notification) -> None:
    """
    Sends an notification notification to the Elastic dashboard.
    The notification is serialized as a JSON string before being published.

    Args:
        notification (ElasticNotification): The notification object containing the notification details.

    Raises:
        Exception: If there is an error during serialization or publishing the notification.
    """
    global __notification_topic
    global __notification_publisher

    if not __notification_topic:
        __notification_topic = NetworkTableInstance.getDefault().getStringTopic(
            "/Elastic/RobotNotifications"
        )
    if not __notification_publisher:
        __notification_publisher = __notification_topic.publish(
            PubSubOptions(sendAll=True, keepDuplicates=True)
        )

    with contextlib.suppress(Exception):
        __notification_publisher.set(
            json.dumps(
                {
                    "level": notification.level.value,
                    "title": notification.title,
                    "description": notification.description,
                    "displayTime": notification.display_time,
                    "width": notification.width,
                    "height": notification.height,
                }
            )
        )


def select_tab(tab_name: str) -> None:
    """
    Selects the tab of the dashboard with the given name.
    If no tab matches the name, this will have no effect on the widgets or tabs in view.
    If the given name is a number, Elastic will select the tab whose index equals the number provided.

    Args:
        tab_name (str) the name of the tab to select
    """
    global __selected_tab_topic
    global __selected_tab_publisher

    if not __selected_tab_topic:
        __selected_tab_topic = NetworkTableInstance.getDefault().getStringTopic(
            "/Elastic/SelectedTab"
        )
    if not __selected_tab_publisher:
        __selected_tab_publisher = __selected_tab_topic.publish(
            PubSubOptions(keepDuplicates=True)
        )

    __selected_tab_publisher.set(tab_name)


def select_tab_index(tab_index: int) -> None:
    """
    Selects the tab of the dashboard at the given index.
    If this index is greater than or equal to the number of tabs, this will have no effect.

    Args:
        tab_index (int) the index of the tab to select
    """
    select_tab(str(tab_index))


class NotificationManager:
    """Helper class for scheduling conditional and unconditional notifications.

    This allows you to define notifications in the constructor of a class without
    having to clutter periodic methods with too many logic checks. It also ensures
    that notifications are properly synchronized so that one notification display
    doesn't overlap with another.
    """

    DEFAULT_TIMEOUT = 30
    SYNC_DELAY = 4  # Seconds between notifications to ensure no overlapping

    # Seconds for network tables to set up, ensures all notifications get sent through
    SETUP_DELAY = 2

    _logger = logging.getLogger("NotificationManager")

    _level_lookup: dict[NotificationLevel, int] = {
        NotificationLevel.INFO: logging.INFO,
        NotificationLevel.WARNING: logging.WARNING,
        NotificationLevel.ERROR: logging.CRITICAL,
    }

    _temp_notifications: dict[Callable[[], bool], "Notification"] = {}
    _persistent_notifications: dict[
        Callable[[], bool], tuple["Notification", list[float]]
    ] = {}

    # Deque is a bit quicker for this purpose
    _pending_notifications: deque["Notification"] = deque()
    _last_send_time: units.seconds = 0
    _last_sent_notification: Notification = Notification()  # For spam stopping purposes

    @staticmethod
    def add_conditional_notification(
        notification: "Notification",
        condition: Callable[[], bool],
        persistent: bool = True,
        cooldown: Optional[float] = None,
    ) -> None:
        """Adds a conditional notification to the manager.

        These notifications are triggered whenever the provided condition is met and
        are automatically synchronized to ensure that notifications do not overlap.
        This is useful for status messages, such as notifying when a device on the CAN network
        is disconnected.

        Args:
            notification: The notification object to display on elastic.
            condition: The condition to attach to the notification. The notification will
                send whenever this condition is satisfied.
            persistent: Whether the notification should remain in circulation after being called.
                Most conditional notifications should be persistent since those conditions can be true
                multiple times per match.
            cooldown (Optional): The minimum cooldown that the persistent notification waits before sending again
                even if the condition is true. If not provided, this defaults to 30 seconds.
        """

        if persistent:
            # [cooldown, last_sent]
            NotificationManager._persistent_notifications[condition] = (
                notification,
                [cooldown or NotificationManager.DEFAULT_TIMEOUT, 0],
            )
        else:
            NotificationManager._temp_notifications[condition] = notification

    @staticmethod
    def _check_equality(
        notification1: Notification, notification2: Notification
    ) -> bool:
        """Checks if two notification objects are the same.

        This makes sure that the title, description, and level of the notifications are the same. So the only
        thing that differs are the display things. This basically ensures that the two notifications contain identical
        content.

        Args:
            notification1: the first notification to check
            notification2: the second notification to check
        """
        return (
            notification1.title == notification2.title
            and notification1.description == notification2.description
            and notification1.level == notification2.level
        )

    @staticmethod
    def add_unconditional_notification(notification: "Notification") -> None:
        """Adds an unconditional notification to the manager.

        This is a shorthand for creating a temporary conditional notification with
        a condition of `lambda: True`, so this will send once at the next opportunity.
        Although it essentially does the same thing as the elastic `send_notification` method,
        it provides the added benefit of synchronizing with the manager's other notifications.

        Args:
            notification: The notification object to display on elastic.
        """
        if not NotificationManager._is_in_queue(notification):
            NotificationManager._temp_notifications[lambda: True] = notification

    @staticmethod
    def send_notifications(limit_notifications: bool) -> None:
        """Sends all notifications in the manager's queue if their condition and timeouts (if applicable) are met.

        Args:
            limit_notifications: let only urgent error notifications be sent through, mostly to avoid distracting
                drivers during competition.
        """
        current_time = Timer.getFPGATimestamp()

        NotificationManager._send_temporary_notifications()
        NotificationManager._send_persistent_notification(current_time)

        NotificationManager._handle_pending_notifications(
            current_time, limit_notifications
        )

    @staticmethod
    def _handle_pending_notifications(
        current_time: units.seconds, limit_notifications: bool
    ) -> None:
        """Handles all notifications in the pending notifications deque, staggering notifications by the sync delay constant.

        Args:
            current_time: the current time according to the FPGA clock
            limit_notifications: let only urgent error notifications be sent through, mostly to avoid distracting
                drivers during competition.
        """
        if (
            not NotificationManager._pending_notifications
            or current_time < NotificationManager.SETUP_DELAY
        ):
            return

        if (
            NotificationManager._last_send_time == 0
            or current_time - NotificationManager._last_send_time
            > NotificationManager.SYNC_DELAY
        ):
            # O(1) v. O(N) with a regular list
            notification = NotificationManager._pending_notifications.popleft()

            if not limit_notifications or notification.level is NotificationLevel.ERROR:
                NotificationManager._logger.log(
                    NotificationManager._level_lookup.get(
                        notification.level, logging.INFO
                    ),
                    f"Sent notification at {current_time:.2f}s: {notification.title} - {notification.description}",
                )
                send_notification(notification)
                NotificationManager._last_send_time = current_time
                NotificationManager._last_sent_notification = notification

    @staticmethod
    def _is_in_queue(notification: Notification) -> bool:
        """Checks if a notification is in the pending notifications deque.

        This can be used to prevent unconditional notification spam. For example, this will ensure that if a driver sends a constant stream of notification inputs for x seconds,
        then only x divided by the sync delay constant will be sent, which although heuristic, will provide more than enough spam protection for
        most use cases.

        Args:
            notification: the notification object to be checked
        """
        # Cannot directly compare memory since most added notifications will be seperate objects with the same content
        return (
            NotificationManager._check_equality(
                notification, NotificationManager._last_sent_notification
            )
            # So notifications from long ago don't block current ones
            and Timer.getFPGATimestamp() - NotificationManager._last_send_time
            < NotificationManager.SYNC_DELAY
        ) or any(
            NotificationManager._check_equality(notification, queued)
            for queued in NotificationManager._pending_notifications
        )

    @staticmethod
    def _send_temporary_notifications() -> None:
        """Appends all temporary notifications whose conditions are met to the pending notifications deque, removing them
        and their conditions from the temporary notifications dictionary.
        """
        if NotificationManager._temp_notifications:
            for condition in list(NotificationManager._temp_notifications):
                if condition():
                    notification = NotificationManager._temp_notifications.pop(
                        condition
                    )
                    NotificationManager._pending_notifications.append(notification)

    @staticmethod
    def _send_persistent_notification(current_time: units.seconds) -> None:
        """Appends all persistent notifications whose conditions and timeout are met to the pending notifications deque.

        Timeouts are based on when the notification is displayed, not when the condition initially is met.

        Args:
            current_time: the current time according to the FPGA clock
        """
        for condition, (
            notification,
            timing,
        ) in NotificationManager._persistent_notifications.items():
            timeout, last_send_time = timing

            if condition() and (
                current_time - last_send_time > timeout or last_send_time == 0
            ):
                NotificationManager._pending_notifications.append(notification)

                # Compensates for notifications already in queue this is done so timings align with on
                # screen displays time, not when the condition becomes true
                stack_time = (
                    len(NotificationManager._pending_notifications)
                    * NotificationManager.SYNC_DELAY
                )
                timing[1] = current_time + stack_time
