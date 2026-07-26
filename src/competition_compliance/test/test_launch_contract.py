#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


WORKSPACE = pathlib.Path(__file__).resolve().parents[3]
MULTI_LAUNCH = WORKSPACE / "robocup_zzufly.launch"
SINGLE_LAUNCH = (
    WORKSPACE
    / "src/competition_compliance/launch/single_vehicle_spawn_clean.launch"
)
CLEAN_INCLUDE = (
    "$(find competition_compliance)/launch/"
    "single_vehicle_spawn_clean.launch"
)


def required_arg(root, name):
    matches = root.findall("./arg[@name='{}']".format(name))
    if len(matches) != 1:
        raise AssertionError(
            "expected exactly one root arg {!r}, found {}".format(name, len(matches))
        )
    return matches[0]


def args_by_name(parent):
    return {
        arg.attrib["name"]: arg.attrib.get("value", arg.attrib.get("default"))
        for arg in parent.findall("arg")
    }


class MultiVehicleLaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = MULTI_LAUNCH.read_text(encoding="utf-8")
        cls.root = ET.fromstring(cls.text)

    def test_requires_explicit_model_file(self):
        model_file = required_arg(self.root, "model_file")
        self.assertEqual({"name": "model_file"}, model_file.attrib)

    def test_spawns_exactly_six_vehicles_with_clean_launcher_and_model(self):
        clean_includes = self.root.findall(".//include[@file='{}']".format(CLEAN_INCLUDE))
        self.assertEqual(6, len(clean_includes))

        model_args = [
            arg
            for include in clean_includes
            for arg in include.findall("arg[@name='sdf_file']")
            if arg.attrib.get("value") == "$(arg model_file)"
        ]
        self.assertEqual(6, len(model_args))
        self.assertEqual(6, self.text.count('name="sdf_file" value="$(arg model_file)"'))

    def test_preserves_all_vehicle_and_mavros_configuration(self):
        expected = (
            ("0", "-17", "-3", "18570", "4560", "13030", "udp://:24540@localhost:34580"),
            ("1", "-14", "-3", "18571", "4561", "13031", "udp://:24541@localhost:34581"),
            ("2", "-17", "0", "18572", "4562", "13032", "udp://:24542@localhost:34582"),
            ("3", "-14", "0", "18573", "4563", "13033", "udp://:24543@localhost:34583"),
            ("4", "-17", "3", "18574", "4564", "13034", "udp://:24544@localhost:34584"),
            ("5", "-14", "3", "18575", "4565", "13035", "udp://:24545@localhost:34585"),
        )

        for vehicle_id, x, y, udp, tcp, gimbal, fcu_url in expected:
            with self.subTest(vehicle=vehicle_id):
                group = self.root.find("./group[@ns='typhoon_h480_{}']".format(vehicle_id))
                self.assertIsNotNone(group)
                group_args = args_by_name(group)
                self.assertEqual(vehicle_id, group_args["ID"])
                self.assertEqual(vehicle_id, group_args["ID_in_group"])
                self.assertEqual(fcu_url, group_args["fcu_url"])

                spawn = group.find("./include[@file='{}']".format(CLEAN_INCLUDE))
                self.assertIsNotNone(spawn)
                spawn_args = args_by_name(spawn)
                self.assertEqual(
                    {
                        "x": x,
                        "y": y,
                        "z": "1",
                        "R": "0",
                        "P": "0",
                        "Y": "0",
                        "vehicle": "typhoon_h480",
                        "sdf_file": "$(arg model_file)",
                        "mavlink_udp_port": udp,
                        "mavlink_tcp_port": tcp,
                        "udp_gimbal_port": gimbal,
                        "ID": "$(arg ID)",
                        "ID_in_group": "$(arg ID_in_group)",
                    },
                    spawn_args,
                )

                mavros = group.find("./include[@file='$(find mavros)/launch/px4.launch']")
                self.assertIsNotNone(mavros)
                self.assertEqual(
                    {
                        "fcu_url": "$(arg fcu_url)",
                        "gcs_url": "",
                        "tgt_system": "$(eval 1 + arg('ID'))",
                        "tgt_component": "1",
                    },
                    args_by_name(mavros),
                )

    def test_does_not_reference_debug_model_or_official_spawn_launcher(self):
        self.assertNotIn("typhoon_h480_zzufly", self.text)
        self.assertNotIn("single_vehicle_spawn_xtd.launch", self.text)


class SingleVehicleLaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = SINGLE_LAUNCH.read_text(encoding="utf-8")
        cls.root = ET.fromstring(cls.text)

    def test_declares_clean_spawn_arguments(self):
        sdf_file = required_arg(self.root, "sdf_file")
        self.assertEqual({"name": "sdf_file"}, sdf_file.attrib)

        expected_defaults = {
            "x": "0",
            "y": "0",
            "z": "0",
            "R": "0",
            "P": "0",
            "Y": "0",
            "est": "ekf2",
            "vehicle": "typhoon_h480",
            "ID": "0",
            "ID_in_group": "0",
            "mavlink_udp_port": "14560",
            "mavlink_tcp_port": "4560",
            "udp_gimbal_port": "13030",
            "interactive": "true",
        }
        for name, default in expected_defaults.items():
            with self.subTest(arg=name):
                self.assertEqual(default, required_arg(self.root, name).attrib.get("default"))

    def test_model_description_reads_only_explicit_file_and_updates_ports(self):
        model_description = self.root.find("./param[@name='model_description']")
        self.assertIsNotNone(model_description)
        command = " ".join(model_description.attrib["command"].split())

        self.assertTrue(command.startswith("xmlstarlet ed "))
        self.assertIn(
            "-d '//plugin[@name=\"mavlink_interface\"]/mavlink_tcp_port'",
            command,
        )
        self.assertIn(
            "-s '//plugin[@name=\"mavlink_interface\"]' -t elem "
            "-n mavlink_tcp_port -v $(arg mavlink_tcp_port)",
            command,
        )
        self.assertIn(
            "-u '//plugin[@name=\"gimbal_controller\"]/udp_gimbal_port_remote' "
            "-v $(arg udp_gimbal_port)",
            command,
        )
        self.assertEqual(1, command.count("$(arg sdf_file)"))
        self.assertTrue(command.endswith("$(arg sdf_file)"))

    def test_never_reads_or_writes_official_model_directories(self):
        self.assertNotIn("Tools/sitl_gazebo/models", self.text)
        self.assertNotIn("ln -s", self.text)
        self.assertNotRegex(self.text, r"\$\(find px4\).*(?:models|\.sdf)")

    def test_configures_px4_environment_and_interactive_mode(self):
        environments = {
            env.attrib["name"]: env.attrib.get("value") for env in self.root.findall("./env")
        }
        self.assertEqual("$(arg vehicle)", environments["PX4_SIM_MODEL"])
        self.assertEqual("$(arg est)", environments["PX4_ESTIMATOR"])

        disabled = self.root.find("./arg[@name='px4_command_arg1'][@unless='$(arg interactive)']")
        enabled = self.root.find("./arg[@name='px4_command_arg1'][@if='$(arg interactive)']")
        self.assertEqual("", disabled.attrib.get("value"))
        self.assertEqual("-d", enabled.attrib.get("value"))

    def test_starts_px4_with_official_romfs_ids_and_workdir(self):
        node = self.root.find("./node[@pkg='px4'][@type='px4']")
        self.assertIsNotNone(node)
        self.assertEqual("sitl_$(arg ID)", node.attrib.get("name"))
        self.assertEqual("screen", node.attrib.get("output"))
        self.assertEqual(
            "$(find px4)/ROMFS/px4fmu_common -s etc/init.d-posix/rcS "
            "-i $(arg ID) -w sitl_$(arg vehicle)_$(arg ID) $(arg px4_command_arg1)",
            " ".join(node.attrib["args"].split()),
        )

    def test_spawns_model_description_at_requested_pose(self):
        node = self.root.find("./node[@pkg='gazebo_ros'][@type='spawn_model']")
        self.assertIsNotNone(node)
        self.assertEqual("$(arg vehicle)_$(arg ID)_spawn", node.attrib.get("name"))
        self.assertEqual("screen", node.attrib.get("output"))
        self.assertEqual(
            "-sdf -param model_description -model $(arg vehicle)_$(arg ID_in_group) "
            "-x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)",
            " ".join(node.attrib["args"].split()),
        )


if __name__ == "__main__":
    unittest.main()
