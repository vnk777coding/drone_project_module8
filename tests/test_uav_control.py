"""импортируем модуль uav_control для тестирования класса UAVControl"""

import unittest
import pytest
from unittest.mock import MagicMock, patch
from pymavlink import mavutil
from uav_control import UAVControl


class TestUAVControl(unittest.TestCase):
    """Тестирует класс UAVControl
    """
    def setUp(self):
        # Создание mock-объекта для mavlink_connection
        self.patcher = patch('uav_control.mavutil.mavlink_connection')
        self.mock_mavlink_connection = self.patcher.start()
        self.mock_master = MagicMock()
        self.mock_mavlink_connection.return_value = self.mock_master
        self.mock_master.wait_heartbeat.return_value = True
        # Настройка mode-mapping словарь режимов полета
        self.mock_master.mode_mapping.return_value = {'GUIDED': 4, 'LAND': 9, 'RTL': 6}

        self.uav = UAVControl('udp:127.0.0.1:14550')

    def tearDown(self):
        # Остановка патчера
        self.patcher.stop()

    def test_connection(self):
        """Метод проверки установки соединения"""
        self.mock_mavlink_connection.assert_called_once_with('udp:127.0.0.1:14550')
        self.mock_master.wait_heartbeat.assert_called_once()

    def test_arm_disarm(self):
        """Проверка запуска и остановки двигателей БПЛА"""
        self.uav.arm()
        self.mock_master.arducopter_arm.accert_called_once()
        self.mock_master.motors_armed_wait().accert_called_once()

        self.uav.disarm()
        self.mock_master.arducopter_disarm().accert_called_once()
        self.mock_master.motors_disarmed_wait().accert_called_once()

    def test_set_mode_valid(self):
        """Проверка установки корректного режима полета"""
        self.uav.set_mode('GUIDED')
        expected_mode_id = self.mock_master.mode_mapping.return_value.get('GUIDED')
        self.mock_master.set_mode.assert_called_with(expected_mode_id)

    def test_set_mode_invalid(self):
        """Проверка реакции на установку некорректного режима"""
        with self.assertRaises(ValueError):
            self.uav.set_mode('INVALIDE_MODE')

    def test_takeoff_positive_altitude(self):
        """Проверяем взлет на положительную высоту"""
        # Настраиваем координаты для возврата
        msg_pos = MagicMock()
        msg_pos.get_type.return_value = 'GLOBAL_POSITION_INT'
        msg_pos.latitude = 550000000  # 55.0 градусов
        msg_pos.longitude = 370000000  # 37.0 градусов
        self.mock_master.recv_match.return_value = msg_pos

        uav = UAVControl.__new__(UAVControl)
        uav.master = self.mock_master
        uav.set_mode = MagicMock()
        uav.wait_command_ack = MagicMock(return_value=True)
        uav.arm = MagicMock()
        uav.seq = 0  # инициализируем последовательность для миссий

        uav.takeoff(10)

        uav.set_mode.assert_called_with('GUIDED')
        uav.arm.assert_called_once()
        self.mock_master.recv_match.assert_called_with(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        self.mock_master.mav.command_long_send.assert_called_once()
        # expected_mode_id = self.mock_master.mode_mapping.return_value.get('GUIDED')
        uav.wait_command_ack.assert_called_with(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    def test_takeoff_negative_altitude(self):
        """Проверка реакции на отрицательную высоту"""
        with self.assertRaises(ValueError):
            self.uav.takeoff(-5)

    def test_goto(self):
        """Проверка команды полета к заданной точке"""
        uav = UAVControl.__new__(UAVControl)
        uav.master = self.mock_master
        uav.wait_command_ack = MagicMock(return_value=True)
        uav.seq = 0  # Инециализируем последовательность для миссий

        uav.goto(55.0, 37.0, 100.0)

        self.mock_master.mav.mission_count_send.assert_called_with(
            uav.master.target_system,
            uav.master.target_component,
            1,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        self.mock_master.mav.mission_item_send.assert_called()
        uav.wait_command_ack.assert_called_with(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)

        # Проверяем с какими параметрами вызывается mission_item_send
        args, kwargs = self.mock_master.mav.mission_item_send.call_args

        # Проверяем что используется правильный фрейм(система) координат
        frame = args[3]
        # self.assertEqual(frame, mavutil.mavlink.MAV_FRAME_GLOBAL_INT)
        self.assertEqual(frame, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def test_get_telemetry(self):
        """Проверка получения телеметрических данных"""
        altitude_msg = MagicMock()
        altitude_msg.get_type.return_value = 'ATTITUDE'
        altitude_msg.roll = 0.1
        altitude_msg.pitch = 0.2
        altitude_msg.yaw = 0.3

        self.mock_master.recv_match.return_value = altitude_msg

        telemetry = self.uav.get_telemetry()

        self.assertIsNotNone(telemetry)
        self.assertEqual(telemetry['roll'], 0.1)
        self.assertEqual(telemetry['pitch'], 0.2)
        self.assertEqual(telemetry['yaw'], 0.3)

    def test_wait_command_ack(self):
        """Проверка ожидания подтверждения команды"""
        ack_msg = MagicMock()
        ack_msg.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        ack_msg.result = mavutil.mavlink.MAV_RESULT_ACCEPTED

        self.mock_master.recv_match.return_value = ack_msg

        result = self.uav.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        self.assertTrue(result)

    def test_wait_command_ack_timeout(self):
        """Проверка таймаута при ожидании подтверждения команды"""
        self.mock_master.recv_match.return_value = None

        result = self.uav.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=1)
        self.assertFalse(result)

    def test_land_success(self):
        """Проверка успешной посадки БПЛА"""
        mock_master = MagicMock()
        mock_master.mode_mapping.return_value = {'LAND': 9}

        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.wait_command_ack = MagicMock(return_value=True)

        uav.land()

        mock_master.set_mode.assert_called_with(9)
        uav.wait_command_ack.assert_called_with(mavutil.mavlink.MAV_CMD_NAV_LAND)
        assert uav.wait_command_ack.call_count == 1

    def test_land_command_ack_failure(self):
        """Проверяет поведение при неприятия команды посадки"""
        mock_master = MagicMock()
        mock_master.mode_mapping.return_value = {'LAND': 9}
        uav = UAVControl.__new__(UAVControl)
        uav.master = mock_master
        uav.wait_command_ack = MagicMock(return_value=False)

        with pytest.raises(RuntimeError, match="Failed to land: Команда для посадки не подтверждена"):
            uav.land()


if __name__ == "__main__":
    unittest.main()
