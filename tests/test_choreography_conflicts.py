from dataclasses import replace
import os
from pathlib import Path
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QMessageBox

from dual_dog.control.choreography import (
    DanceStep, MoveDirection, PhaseKind, build_multitrack_dance,
    overlapping_dance_cue_messages, resolve_dance_time_conflicts,
)
from dual_dog.config import load_config
from dual_dog.ui.dance_editor import ChoreographyEditorDialog
from dual_dog.models import Command


TARGETS = ("dog01", "dog02", "dog03")


class ConflictResolutionTests(unittest.TestCase):
    def resolve(self, steps, lock=4.0):
        result = resolve_dance_time_conflicts(steps, lock, TARGETS)
        self.assertEqual(len(result), len(steps))
        for before, after in zip(steps, result):
            self.assertEqual(before, replace(after, start_time=before.start_time))
            if before.start_time is not None:
                self.assertGreaterEqual(after.start_time, before.start_time)
        phases = build_multitrack_dance(result, lock, TARGETS, post_action_correction_seconds=3.0)
        self.assertFalse(overlapping_dance_cue_messages(phases))
        self.assertEqual(resolve_dance_time_conflicts(result, lock, TARGETS), result)
        return result

    def test_hidden_corrections_delay_only_affected_track(self):
        steps = [
            DanceStep(PhaseKind.TRICK, 2.0, Command.PRONK, start_time=0, targets=("dog01",)),
            DanceStep(PhaseKind.TRICK, 2.0, Command.PACE, start_time=2, targets=("dog01",)),
            DanceStep(PhaseKind.POSTURE, 2.0, Command.SIT, start_time=4, targets=("dog01",)),
            DanceStep(PhaseKind.MOVE, 8.0, speed=0.2, start_time=0, targets=("dog02",),
                      direction=MoveDirection.LEFT),
        ]
        self.assertEqual([s.start_time for s in self.resolve(steps)], [0, 5, 10, 0])

    def test_multi_target_and_default_targets_remain_intact(self):
        steps = [
            DanceStep(PhaseKind.POSTURE, 4, Command.STAND, start_time=0, targets=("dog01",)),
            DanceStep(PhaseKind.POSTURE, 2, Command.STAND, start_time=0, targets=("dog02",)),
            DanceStep(PhaseKind.MOVE, 2, speed=0.2, start_time=1, targets=("dog01", "dog02")),
            DanceStep(PhaseKind.POSTURE, 2, Command.SIT, start_time=3, targets=()),
        ]
        self.assertEqual([s.start_time for s in self.resolve(steps)], [0, 0, 4, 6])

    def test_flip_lock_and_uwb_return_are_reserved_without_changing_duration(self):
        for command, duration, expected in [(Command.LEFT_FLIP, 0.8, 11), (Command.PACE, 2, 13)]:
            with self.subTest(command=command):
                steps = [
                    DanceStep(PhaseKind.TRICK, duration, command, start_time=0,
                              targets=("dog01",), return_to_start=True),
                    DanceStep(PhaseKind.STOP, 1, start_time=1, targets=("dog01",)),
                ]
                self.assertEqual(self.resolve(steps)[1].start_time, expected)

    def test_existing_correction_uses_minimum_runtime_duration(self):
        steps = [
            DanceStep(PhaseKind.TRICK, 2, Command.PRONK, start_time=0, targets=("dog01",)),
            DanceStep(PhaseKind.CORRECT_YAW, 1, start_time=2, targets=("dog01",)),
            DanceStep(PhaseKind.POSTURE, 2, Command.SIT, start_time=3, targets=("dog01",)),
        ]
        with self.assertRaisesRegex(ValueError, "时间重叠"):
            build_multitrack_dance(steps, 4, TARGETS, post_action_correction_seconds=3)
        self.assertEqual([s.start_time for s in self.resolve(steps)], [0, 2, 5])

    def test_moving_explicit_correction_can_require_new_hidden_correction(self):
        steps = [
            DanceStep(PhaseKind.TRICK, 2, Command.PRONK, start_time=0, targets=("dog01",)),
            DanceStep(PhaseKind.MOVE, 4, speed=0.2, start_time=0, targets=("dog01",)),
            DanceStep(PhaseKind.CORRECT_YAW, 1, start_time=2, targets=("dog01",)),
            DanceStep(PhaseKind.STOP, 1, start_time=3, targets=("dog01",)),
        ]
        result = self.resolve(steps)
        self.assertEqual(result[1].start_time, 5)
        self.assertGreaterEqual(result[2].start_time, 9)

    def test_unsorted_rows_keep_original_chronological_order_and_gaps(self):
        steps = [
            DanceStep(PhaseKind.POSTURE, 2, Command.SIT, start_time=20, targets=("dog01",)),
            DanceStep(PhaseKind.TRICK, 2, Command.PRONK, start_time=0, targets=("dog01",)),
            DanceStep(PhaseKind.POSTURE, 2, Command.STAND, start_time=2, targets=("dog01",)),
        ]
        self.assertEqual([s.start_time for s in self.resolve(steps)], [20, 0, 5])

    def test_implicit_starts_and_fractional_rounding(self):
        steps = [
            DanceStep(PhaseKind.TRICK, 0.555, Command.PACE, targets=("dog01",)),
            DanceStep(PhaseKind.STOP, 1, targets=("dog01",)),
        ]
        self.assertEqual([s.start_time for s in self.resolve(steps)], [0, 3.56])

    def test_conflict_free_document_is_unchanged(self):
        steps = [DanceStep(PhaseKind.TRICK, 2, Command.PRONK, targets=("dog01",))]
        self.assertEqual(self.resolve(steps), steps)


class EditorConflictResolutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_button_only_updates_start_cells_and_supports_long_timelines(self):
        root = Path(__file__).resolve().parents[1]
        _, robots, safety = load_config(root / "config/robots.json")
        steps = [
            DanceStep(PhaseKind.TRICK, 10, Command.PRONK, start_time=595, targets=("dog01",)),
            DanceStep(PhaseKind.POSTURE, 2, Command.SIT, start_time=600, targets=("dog01",)),
        ]
        dialog = ChoreographyEditorDialog(steps, robots, safety.max_x, safety.max_y,
                                          safety.max_yaw, 5, 4)
        try:
            before = dialog.steps()
            dialog.resolve_conflicts_button.click()
            after = dialog.steps()
            self.assertEqual(after[1].start_time, 608)
            for old, new in zip(before, after):
                self.assertEqual(old, replace(new, start_time=old.start_time))
            dialog.resolve_conflicts_button.click()
            self.assertEqual(after, dialog.steps())
            dialog.rebuild(after, 0)
            self.assertEqual(after, dialog.steps())
        finally:
            dialog.close()

    def test_invalid_action_leaves_all_cells_unchanged(self):
        root = Path(__file__).resolve().parents[1]
        _, robots, safety = load_config(root / "config/robots.json")
        dialog = ChoreographyEditorDialog([], robots, safety.max_x, safety.max_y,
                                          safety.max_yaw, 5, 4)
        try:
            dialog.add_step(PhaseKind.TRICK, Command.PRONK)
            dialog.table.cellWidget(0, 4).setValue(20)
            before = dialog.steps()
            with patch.object(QMessageBox, "warning") as warning:
                dialog.resolve_conflicts_button.click()
            warning.assert_called_once()
            self.assertEqual(dialog.steps(), before)
        finally:
            dialog.close()


if __name__ == "__main__":
    unittest.main()
