from dataclasses import replace
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QFileDialog, QMessageBox

from dual_dog.control.choreography import (
    DanceStep, PhaseKind, build_multitrack_dance, dance_steps_from_payload,
    dance_steps_to_payload, merge_dance_documents, overlapping_dance_cue_messages,
)
from dual_dog.config import load_config
from dual_dog.ui.dance_editor import ChoreographyEditorDialog
from dual_dog.models import Command


def stand(robot, start=0):
    return DanceStep(PhaseKind.POSTURE, 2, Command.STAND, start_time=start, targets=(robot,))


class MergeDocumentsTests(unittest.TestCase):
    def test_independent_timelines_and_current_steps_remain_unchanged(self):
        current = [stand("dog01", 20)]
        documents = [("dog02.json", dance_steps_to_payload([stand("dog02", 0)])),
                     ("dog03.json", dance_steps_to_payload([stand("dog03", 0)]))]
        merged = merge_dance_documents(current, documents, 4, ["dog01", "dog02", "dog03"])
        self.assertEqual(merged, [stand("dog01", 20), stand("dog02", 0), stand("dog03", 0)])
        self.assertEqual(current, [stand("dog01", 20)])
        self.assertEqual(dance_steps_from_payload(dance_steps_to_payload(merged)), merged)

    def test_implicit_times_resolve_per_file_not_after_previous_file(self):
        docs = []
        for robot in ("dog01", "dog02"):
            docs.append((robot, dance_steps_to_payload([
                stand(robot, None), replace(stand(robot, None), command=Command.SIT),
            ])))
        merged = merge_dance_documents([], docs, 4, ["dog01", "dog02"])
        self.assertEqual([step.start_time for step in merged], [0, 2, 0, 2])

    def test_unknown_robot_rejects_batch_without_mutating_source(self):
        current = [stand("dog01")]
        documents = [("valid", dance_steps_to_payload([stand("dog01")])),
                     ("bad.json", dance_steps_to_payload([stand("dog99")]))]
        with self.assertRaisesRegex(ValueError, "bad.json"):
            merge_dance_documents(current, documents, 4, ["dog01"])
        self.assertEqual(current, [stand("dog01")])

    def test_combined_limit_is_enforced(self):
        with self.assertRaisesRegex(ValueError, "1000"):
            merge_dance_documents([stand("dog01")] * 1000,
                                  [("one.json", dance_steps_to_payload([stand("dog01")]))],
                                  4, ["dog01"])


class EditorImportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        root = Path(__file__).resolve().parents[1]
        _, cls.robots, cls.safety = load_config(root / "config/robots.json")

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.dialog = ChoreographyEditorDialog([stand("dog01", 10)], self.robots,
            self.safety.max_x, self.safety.max_y, self.safety.max_yaw, 5, 4)
        self.addCleanup(self.dialog.close)

    def write(self, name, steps):
        path = Path(self.folder.name) / name
        payload = dance_steps_to_payload(steps)
        payload["music"] = "keep-source-music.wav"
        path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8-sig")
        return str(path)

    def select(self, files):
        with patch.object(QFileDialog, "getOpenFileNames", return_value=(files, "")), \
                patch.object(QMessageBox, "warning") as warning:
            self.dialog.import_dances_button.click()
        return warning

    def test_multiselect_preserves_existing_edits_and_source_files(self):
        self.dialog.table.cellWidget(0, 0).setValue(12.5)
        before = self.dialog.steps()
        files = [self.write("dog02.json", [stand("dog02")]),
                 self.write("dog03.json", [stand("dog03")])]
        original_bytes = [Path(name).read_bytes() for name in files]
        warning = self.select(files)
        warning.assert_not_called()
        self.assertEqual(self.dialog.steps(), before + [stand("dog02"), stand("dog03")])
        self.assertEqual([Path(name).read_bytes() for name in files], original_bytes)
        saved = dance_steps_to_payload(self.dialog.steps())
        self.assertEqual(dance_steps_from_payload(saved), self.dialog.steps())
        self.assertIn("2 个方案", self.dialog.conflict_status.text())

    def test_invalid_second_file_rolls_back_entire_batch(self):
        before = self.dialog.steps()
        valid = self.write("valid.json", [stand("dog02")])
        bad = Path(self.folder.name) / "bad.json"
        bad.write_text("{ invalid json", encoding="utf-8")
        self.select([valid, str(bad)]).assert_called_once()
        self.assertEqual(self.dialog.steps(), before)

    def test_unknown_robot_does_not_become_all_selected_devices(self):
        before = self.dialog.steps()
        self.select([self.write("unknown.json", [stand("dog99")])]).assert_called_once()
        self.assertEqual(self.dialog.steps(), before)

    def test_unsupported_speed_rolls_back_appended_rows(self):
        before = self.dialog.steps()
        invalid = DanceStep(PhaseKind.MOVE, 2, speed=9, start_time=0, targets=("dog02",))
        self.select([self.write("valid.json", [stand("dog02")]),
                     self.write("speed.json", [invalid])]).assert_called_once()
        self.assertEqual(self.dialog.steps(), before)

    def test_hidden_conflicts_reported_and_resolved_after_merge(self):
        self.dialog.rebuild([], 0)
        trick = DanceStep(PhaseKind.TRICK, 2, Command.PRONK, start_time=0, targets=("dog01",))
        self.select([self.write("trick.json", [trick]),
                     self.write("posture.json", [stand("dog01", 2)])]).assert_not_called()
        self.assertEqual(self.dialog.steps()[1].start_time, 2)
        self.assertIn("时间冲突", self.dialog.conflict_status.text())
        self.dialog.resolve_conflicts_button.click()
        self.assertEqual(self.dialog.steps()[1].start_time, 5)
        compiled = build_multitrack_dance(self.dialog.steps(), 4,
            [r.robot_id for r in self.robots], post_action_correction_seconds=3)
        self.assertFalse(overlapping_dance_cue_messages(compiled))

    def test_flip_duration_legacy_forward_and_long_start_are_preserved(self):
        self.dialog.rebuild([], 0)
        steps = [DanceStep(PhaseKind.TRICK, 1, Command.LEFT_FLIP, start_time=650,
                          targets=("dog02",), return_to_start=True),
                 DanceStep(PhaseKind.FORWARD, 2, speed=0.2, start_time=670, targets=("dog02",))]
        self.select([self.write("legacy.json", steps)]).assert_not_called()
        self.assertEqual(self.dialog.steps(), steps)

    def test_default_targets_remain_default_and_are_identified(self):
        default = replace(stand("dog02"), targets=())
        self.select([self.write("default.json", [default])]).assert_not_called()
        self.assertEqual(self.dialog.steps()[-1], default)
        self.assertIn("所有当前选中设备", self.dialog.conflict_status.text())

    def test_cancel_and_duplicate_file_selection(self):
        before = self.dialog.steps()
        self.select([]).assert_not_called()
        self.assertEqual(self.dialog.steps(), before)
        file = self.write("one.json", [stand("dog02")])
        self.select([file, file]).assert_not_called()
        self.assertEqual(self.dialog.steps(), before + [stand("dog02")])


if __name__ == "__main__":
    unittest.main()
