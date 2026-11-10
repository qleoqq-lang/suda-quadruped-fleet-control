import os
from pathlib import Path
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from dual_dog.control.choreography import DanceStep, PhaseKind
from dual_dog.config import load_config
from dual_dog.control.controller import effective_flip_lock_seconds
from dual_dog.ui.dance_editor import ChoreographyEditorDialog
from dual_dog.models import Command


ROOT = Path(__file__).resolve().parents[1]


class DanceEditorTagTargetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_tag_target_coordinates_survive_editor_round_trip(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        source = DanceStep(
            PhaseKind.TAG_TARGET,
            duration=14.0,
            speed=160.0,
            start_time=2.5,
            targets=(configs[0].robot_id,),
            target_x=1.25,
            target_y=-0.75,
            target_name="进场队形/TAG1",
        )
        dialog = ChoreographyEditorDialog(
            [source],
            configs,
            safety.max_x,
            safety.max_y,
            safety.max_yaw,
            safety.non_flip_action_seconds,
            safety.one_shot_return_ms / 1000.0,
        )
        try:
            self.assertEqual(dialog.table.columnCount(), 10)
            self.assertEqual(dialog.steps(), [source])
        finally:
            dialog.close()

    def test_stand_and_sit_are_available_and_survive_editor_round_trip(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        sources = [
            DanceStep(
                PhaseKind.POSTURE,
                duration=3.0,
                command=Command.STAND,
                start_time=0.0,
                targets=(configs[0].robot_id,),
            ),
            DanceStep(
                PhaseKind.POSTURE,
                duration=2.0,
                command=Command.SIT,
                targets=(configs[1].robot_id,),
                start_time=3.0,
            ),
        ]
        dialog = ChoreographyEditorDialog(
            sources,
            configs,
            safety.max_x,
            safety.max_y,
            safety.max_yaw,
            safety.non_flip_action_seconds,
            safety.one_shot_return_ms / 1000.0,
        )
        try:
            self.assertEqual(dialog.steps(), sources)
            self.assertEqual(dialog.table.cellWidget(0, 3).currentData(), "stand")
            self.assertEqual(dialog.table.cellWidget(1, 3).currentData(), "sit")
        finally:
            dialog.close()

    def test_multiple_robots_can_be_selected_for_one_instruction(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        targets = (configs[0].robot_id, configs[2].robot_id, configs[5].robot_id)
        source = DanceStep(
            PhaseKind.POSTURE,
            duration=4.0,
            command=Command.STAND,
            start_time=1.5,
            targets=targets,
        )
        dialog = ChoreographyEditorDialog(
            [source], configs, safety.max_x, safety.max_y, safety.max_yaw,
            safety.non_flip_action_seconds,
            safety.one_shot_return_ms / 1000.0,
        )
        try:
            selector = dialog.table.cellWidget(0, 1)
            self.assertEqual(selector.targets(), targets)
            self.assertEqual(dialog.steps(), [source])

            selector.set_targets((configs[1].robot_id, configs[3].robot_id))
            self.assertEqual(
                dialog.steps()[0].targets,
                (configs[1].robot_id, configs[3].robot_id),
            )
        finally:
            dialog.close()

    def test_start_time_is_estimated_after_flip_and_remains_editable(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        dialog = ChoreographyEditorDialog(
            [], configs, safety.max_x, safety.max_y, safety.max_yaw,
            safety.non_flip_action_seconds,
            effective_flip_lock_seconds(safety),
        )
        try:
            dialog.add_step(PhaseKind.TRICK, command=Command.LEFT_FLIP)
            dialog.add_step(PhaseKind.POSTURE, command=Command.SIT)
            self.assertEqual(dialog.table.cellWidget(0, 0).value(), 0.0)
            self.assertEqual(dialog.table.cellWidget(1, 0).value(), 7.0)

            dialog.table.cellWidget(1, 0).setValue(9.5)
            self.assertEqual(dialog.steps()[1].start_time, 9.5)

            dialog.auto_arrange_start_times()
            self.assertEqual(dialog.steps()[1].start_time, 7.0)
        finally:
            dialog.close()

    def test_new_choreography_instruction_defaults_to_first_ten_robots(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        dialog = ChoreographyEditorDialog(
            [], configs, safety.max_x, safety.max_y, safety.max_yaw,
            safety.non_flip_action_seconds,
            effective_flip_lock_seconds(safety),
        )
        try:
            dialog.add_step(PhaseKind.POSTURE, command=Command.STAND)
            self.assertEqual(
                dialog.steps()[0].targets,
                tuple(config.robot_id for config in configs[:10]),
            )
        finally:
            dialog.close()


if __name__ == "__main__":
    unittest.main()
