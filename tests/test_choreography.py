import unittest

from dual_dog.control.choreography import (
    DanceStep,
    MoveDirection,
    PhaseKind,
    TRICK_POST_ACTION_CORRECTION_SECONDS,
    build_custom_dance,
    build_flip_forward_dance,
    build_multitrack_dance,
    dance_steps_from_payload,
    dance_steps_to_payload,
    keep_connected_dance_phases,
)
from dual_dog.models import Command


class ChoreographyTests(unittest.TestCase):
    def test_keep_connected_dance_phases_drops_offline_targets(self):
        phases = (
            DanceStep(
                PhaseKind.POSTURE,
                duration=3.0,
                command=Command.STAND,
                start_time=0.0,
                targets=("dog01", "dog02", "dog03"),
            ),
            DanceStep(
                PhaseKind.POSTURE,
                duration=2.0,
                command=Command.SIT,
                start_time=3.0,
                targets=("dog04",),
            ),
        )
        compiled = build_multitrack_dance(phases, 3.0, ("dog01", "dog02", "dog03", "dog04"))

        kept = keep_connected_dance_phases(compiled, {"dog01", "dog03"})

        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0].targets, ("dog01", "dog03"))
        self.assertEqual(kept[0].command, Command.STAND)
        self.assertEqual(kept[0].start_time, 0.0)
        self.assertEqual(keep_connected_dance_phases(compiled, set()), ())

    def test_requested_flip_correct_forward_sequence(self):
        phases = build_flip_forward_dance(2.0, 4.0, 3.0, 0.5)
        self.assertEqual(
            [phase.kind for phase in phases],
            [
                PhaseKind.TRICK,
                PhaseKind.CORRECT_YAW,
                PhaseKind.FORWARD,
                PhaseKind.STOP,
                PhaseKind.TRICK,
                PhaseKind.CORRECT_YAW,
                PhaseKind.FORWARD,
                PhaseKind.STOP,
            ],
        )
        self.assertEqual(phases[0].command, Command.LEFT_FLIP)
        self.assertEqual(phases[4].command, Command.RIGHT_FLIP)
        self.assertEqual([phases[1].duration, phases[5].duration], [2.0, 2.0])
        self.assertEqual([phases[2].duration, phases[6].duration], [4.0, 4.0])
        self.assertEqual([phases[2].speed, phases[6].speed], [0.5, 0.5])

    def test_correct_yaw_after_trick_runs_at_least_three_seconds(self):
        phases = build_multitrack_dance(
            [
                DanceStep(
                    PhaseKind.TRICK,
                    duration=4.0,
                    command=Command.LEFT_FLIP,
                    targets=("dog01",),
                ),
                DanceStep(
                    PhaseKind.CORRECT_YAW,
                    duration=1.0,
                    targets=("dog01",),
                ),
            ],
            3.0,
            ("dog01",),
            allow_overlap=True,
            post_action_correction_seconds=TRICK_POST_ACTION_CORRECTION_SECONDS,
        )
        self.assertEqual(len(phases), 2)
        self.assertEqual(phases[0].kind, PhaseKind.TRICK)
        self.assertEqual(phases[1].kind, PhaseKind.CORRECT_YAW)
        self.assertEqual(phases[1].duration, 3.0)

    def test_trick_automatically_inserts_three_second_correction(self):
        phases = build_multitrack_dance(
            [
                DanceStep(
                    PhaseKind.TRICK,
                    duration=2.0,
                    command=Command.UP_JUMP,
                    start_time=1.0,
                    targets=("dog01",),
                ),
            ],
            3.0,
            ("dog01",),
            allow_overlap=True,
            post_action_correction_seconds=TRICK_POST_ACTION_CORRECTION_SECONDS,
        )

        self.assertEqual([phase.kind for phase in phases], [
            PhaseKind.TRICK,
            PhaseKind.CORRECT_YAW,
        ])
        self.assertEqual(phases[1].start_time, 3.0)
        self.assertEqual(phases[1].duration, 3.0)
        self.assertEqual(phases[1].targets, ("dog01",))

    def test_invalid_durations_are_rejected(self):
        with self.assertRaises(ValueError):
            build_flip_forward_dance(0, 1, 3)
        with self.assertRaises(ValueError):
            build_flip_forward_dance(1, -1, 3)

    def test_actions_and_speed_are_customizable(self):
        phases = build_flip_forward_dance(
            1.5, 2.5, 3.0, 0.65,
            first_command=Command.PRONK,
            second_command=Command.BOUND,
            non_flip_action_seconds=4.0,
        )
        self.assertEqual(phases[0].command, Command.PRONK)
        self.assertEqual(phases[4].command, Command.BOUND)
        self.assertEqual(phases[2].speed, 0.65)
        self.assertEqual(phases[6].speed, 0.65)
        self.assertEqual(phases[0].command_duration, 4.0)
        self.assertEqual(phases[4].command_duration, 4.0)

    def test_sdk_jump_actions_are_allowed(self):
        commands = (
            Command.UP_JUMP,
            Command.FRONT_JUMP,
            Command.BACK_JUMP,
            Command.LEFT_JUMP,
            Command.RIGHT_JUMP,
            Command.UPRIGHT,
        )
        for command in commands:
            phases = build_custom_dance(
                [DanceStep(PhaseKind.TRICK, 2.0, command)], 3.0
            )
            self.assertEqual(phases[0].command, command)

    def test_flips_ignore_non_flip_hold_time(self):
        phases = build_flip_forward_dance(1, 1, 3, non_flip_action_seconds=8.0)
        self.assertEqual(phases[0].command_duration, 0.0)
        self.assertEqual(phases[4].command_duration, 0.0)

    def test_arbitrary_multi_step_order_is_preserved(self):
        steps = [
            DanceStep(PhaseKind.CORRECT_YAW, 1.0),
            DanceStep(PhaseKind.TRICK, 2.5, Command.PACE),
            DanceStep(PhaseKind.STOP, 0.7),
            DanceStep(PhaseKind.FORWARD, 4.0, speed=0.35),
            DanceStep(PhaseKind.TRICK, command=Command.LEFT_FLIP),
            DanceStep(PhaseKind.CORRECT_YAW, 3.0),
        ]
        phases = build_custom_dance(steps, 3.0)
        self.assertEqual([phase.kind for phase in phases], [step.kind for step in steps])
        self.assertEqual(phases[1].command, Command.PACE)
        self.assertEqual(phases[1].command_duration, 2.5)
        self.assertEqual(phases[3].speed, 0.35)
        self.assertEqual(phases[4].command, Command.LEFT_FLIP)

    def test_different_tracks_can_run_concurrently(self):
        phases = build_multitrack_dance(
            [
                DanceStep(
                    PhaseKind.MOVE, 2.0, speed=0.2, start_time=0.0,
                    targets=("dog01",), direction=MoveDirection.LEFT,
                ),
                DanceStep(
                    PhaseKind.MOVE, 2.0, speed=0.2, start_time=0.0,
                    targets=("dog02",), direction=MoveDirection.RIGHT,
                ),
            ],
            3.0,
            ("dog01", "dog02"),
        )
        self.assertEqual([phase.start_time for phase in phases], [0.0, 0.0])
        self.assertGreater(phases[0].y, 0)
        self.assertLess(phases[1].y, 0)

    def test_same_track_overlap_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "时间重叠"):
            build_multitrack_dance(
                [
                    DanceStep(
                        PhaseKind.MOVE, 2.0, speed=0.2, start_time=0.0,
                        targets=("dog01",),
                    ),
                    DanceStep(
                        PhaseKind.MOVE, 1.0, speed=0.2, start_time=1.0,
                        targets=("dog01",),
                    ),
                ],
                3.0,
                ("dog01",),
            )

    def test_same_track_overlap_can_be_allowed(self):
        phases = build_multitrack_dance(
            [
                DanceStep(
                    PhaseKind.MOVE, 2.0, speed=0.2, start_time=0.0,
                    targets=("dog01",),
                ),
                DanceStep(
                    PhaseKind.MOVE, 1.0, speed=0.2, start_time=1.0,
                    targets=("dog01",),
                ),
            ],
            3.0,
            ("dog01",),
            allow_overlap=True,
        )
        self.assertEqual([phase.start_time for phase in phases], [0.0, 1.0])

    def test_non_flip_action_uses_visible_duration_for_track_occupancy(self):
        phases = build_multitrack_dance(
            [
                DanceStep(
                    PhaseKind.TRICK,
                    duration=2.0,
                    command=Command.PRONK,
                    start_time=0.0,
                    targets=("dog01",),
                ),
                DanceStep(
                    PhaseKind.MOVE,
                    duration=1.0,
                    speed=0.2,
                    start_time=2.0,
                    targets=("dog01",),
                ),
            ],
            3.0,
            ("dog01",),
        )
        self.assertEqual([phase.duration for phase in phases], [2.0, 1.0])

    def test_flip_keeps_visible_safety_occupancy(self):
        overlapping_steps = [
            DanceStep(
                PhaseKind.TRICK,
                command=Command.LEFT_FLIP,
                start_time=0.0,
                targets=("dog01",),
            ),
            DanceStep(
                PhaseKind.MOVE,
                duration=1.0,
                speed=0.2,
                start_time=2.0,
                targets=("dog01",),
            ),
        ]
        with self.assertRaisesRegex(ValueError, r"\[0\.00, 3\.00\)"):
            build_multitrack_dance(overlapping_steps, 3.0, ("dog01",))

        adjacent_steps = list(overlapping_steps)
        adjacent_steps[1] = DanceStep(
            PhaseKind.MOVE,
            duration=1.0,
            speed=0.2,
            start_time=3.0,
            targets=("dog01",),
        )
        phases = build_multitrack_dance(adjacent_steps, 3.0, ("dog01",))
        self.assertEqual([phase.duration for phase in phases], [3.0, 1.0])

    def test_empty_track_expands_to_selected_devices(self):
        phases = build_multitrack_dance(
            [DanceStep(PhaseKind.STOP, 0.5, start_time=0.0)],
            3.0,
            ("dog01", "dog02", "dog03"),
        )
        self.assertEqual(phases[0].targets, ("dog01", "dog02", "dog03"))

    def test_choreography_json_payload_round_trip(self):
        steps = [
            DanceStep(
                PhaseKind.MOVE,
                duration=1.25,
                speed=0.12,
                start_time=0.5,
                targets=("dog01", "dog02"),
                direction=MoveDirection.LEFT,
            ),
            DanceStep(
                PhaseKind.TRICK,
                duration=2.0,
                command=Command.PRONK,
                start_time=2.0,
                targets=("dog03",),
            ),
        ]
        self.assertEqual(dance_steps_from_payload(dance_steps_to_payload(steps)), steps)

    def test_tag_target_round_trip_and_compile(self):
        step = DanceStep(
            PhaseKind.TAG_TARGET,
            duration=12.0,
            speed=150.0,
            start_time=3.0,
            targets=("dog03",),
            target_x=2.35,
            target_y=-0.40,
        )
        loaded = dance_steps_from_payload(dance_steps_to_payload([step]))
        self.assertEqual(loaded, [step])
        phase = build_multitrack_dance(loaded, 3.0, ("dog03",))[0]
        self.assertIs(phase.kind, PhaseKind.TAG_TARGET)
        self.assertEqual((phase.x, phase.y), (2.35, -0.40))
        self.assertEqual(phase.speed, 150.0)
        self.assertIn("TAG目标点", phase.label)

    def test_stand_and_sit_round_trip_and_compile_as_postures(self):
        steps = [
            DanceStep(
                PhaseKind.POSTURE,
                duration=3.0,
                command=Command.STAND,
                start_time=0.0,
                targets=("dog01",),
            ),
            DanceStep(
                PhaseKind.POSTURE,
                duration=2.0,
                command=Command.SIT,
                start_time=3.0,
                targets=("dog01",),
            ),
        ]
        loaded = dance_steps_from_payload(dance_steps_to_payload(steps))
        self.assertEqual(loaded, steps)
        phases = build_multitrack_dance(loaded, 3.0, ("dog01",))
        self.assertEqual(
            [phase.command for phase in phases],
            [Command.STAND, Command.SIT],
        )
        self.assertEqual([phase.kind for phase in phases], [PhaseKind.POSTURE] * 2)
        self.assertIn("起立", phases[0].label)
        self.assertIn("坐下", phases[1].label)

    def test_invalid_posture_action_or_duration_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "姿态动作"):
            build_custom_dance(
                [DanceStep(PhaseKind.POSTURE, 2.0, Command.WALK)], 3.0
            )
        with self.assertRaisesRegex(ValueError, "0.5～15"):
            build_custom_dance(
                [DanceStep(PhaseKind.POSTURE, 0.1, Command.STAND)], 3.0
            )

    def test_tag_targets_for_different_dogs_can_start_together(self):
        phases = build_multitrack_dance(
            [
                DanceStep(
                    PhaseKind.TAG_TARGET, 10.0, speed=100.0,
                    start_time=0.0, targets=("dog01",), target_x=1.0,
                ),
                DanceStep(
                    PhaseKind.TAG_TARGET, 10.0, speed=100.0,
                    start_time=0.0, targets=("dog02",), target_y=1.0,
                ),
            ],
            3.0,
            ("dog01", "dog02"),
        )
        self.assertEqual([phase.start_time for phase in phases], [0.0, 0.0])

    def test_invalid_choreography_payload_is_rejected(self):
        invalid_documents = [
            {},
            {"version": 2, "steps": []},
            {"version": 1, "steps": []},
            {"version": 1, "steps": [{"kind": "unknown"}]},
            {
                "version": 1,
                "steps": [{"kind": "stop", "duration": float("nan")}],
            },
        ]
        for payload in invalid_documents:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                dance_steps_from_payload(payload)

    def test_stunt_return_to_start_round_trips_and_reserves_recovery_time(self):
        step = DanceStep(
            PhaseKind.TRICK,
            command=Command.LEFT_FLIP,
            start_time=0.0,
            targets=("dog01",),
            return_to_start=True,
        )
        loaded = dance_steps_from_payload(dance_steps_to_payload([step]))
        self.assertEqual(loaded, [step])
        phase = build_multitrack_dance(loaded, 3.0, ("dog01",))[0]
        self.assertTrue(phase.return_to_start)
        self.assertGreaterEqual(phase.duration, 7.0)
        self.assertIn("动作后回标零", phase.label)


if __name__ == "__main__":
    unittest.main()
