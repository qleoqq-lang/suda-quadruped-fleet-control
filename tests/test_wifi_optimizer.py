import unittest
from unittest.mock import Mock, patch

from dual_dog.ui.network_panel import NetworkPanel
from dual_dog.devices.wifi_optimizer import (
    change_and_verify,
    parse_scan,
    rank_channels,
    reception_unsafe,
    worth_switching,
)


class WifiOptimizerTests(unittest.TestCase):
    def test_parse_bilingual_scan_keeps_each_bssid_and_requires_signal(self):
        result = parse_scan("""
SSID 1 : Router
 BSSID 1 : 50:FE:39:7B:60:F7
 Signal : 90%
 Channel : 36
SSID 2 : Other
 BSSID 1 : AA:BB:CC:DD:EE:FF
 信号 : 30%
 信道 : 149
 BSSID 2 : AA:BB:CC:DD:EE:00
 信道 : 153
""")
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["channel"], 36)
        self.assertEqual(result[1]["signal_percent"], 30)

    def test_strong_24ghz_hotspot_does_not_penalize_5ghz(self):
        networks = [dict(ssid="BPX", channel=1, signal_percent=100)]
        ranking = rank_channels(networks, "Router", (36, 44, 149, 157))
        self.assertTrue(all(row["score"] == 0 for row in ranking))
        self.assertFalse(worth_switching(36, ranking))

    def test_ranking_accounts_for_wide_channel_overlap_and_router_support(self):
        networks = [dict(ssid="Router", channel=36, signal_percent=95),
                    dict(ssid="Other", channel=153, signal_percent=60)]
        ranking = rank_channels(networks, "Router", (36, 149))
        self.assertEqual([row["channel"] for row in ranking], [36, 149])
        self.assertEqual(ranking[0]["score"], 0)
        self.assertEqual(ranking[1]["score"], 36)
        self.assertTrue(worth_switching(149, ranking))

    def test_small_improvement_and_unknown_current_channel_do_not_switch(self):
        ranking = [dict(channel=36, score=8), dict(channel=149, score=10)]
        self.assertFalse(worth_switching(149, ranking))
        self.assertFalse(worth_switching(52, ranking))

    def test_nonempty_candidate_never_replaces_noisy_current_channel(self):
        ranking = [
            dict(channel=149, score=4.41, neighbors=1),
            dict(channel=36, score=131.92, neighbors=2),
        ]
        self.assertFalse(worth_switching(36, ranking))

    def test_only_fully_empty_candidate_can_trigger_switch(self):
        ranking = [
            dict(channel=149, score=0.0, neighbors=0),
            dict(channel=36, score=131.92, neighbors=2),
        ]
        self.assertTrue(worth_switching(36, ranking))

    def test_manual_selection_chooses_empty_channel(self):
        panel = NetworkPanel.__new__(NetworkPanel)
        panel.status = Mock()
        panel.append = Mock()
        panel.last_change = 0.0
        panel.pending_candidate = None
        panel.manual_select_requested = True
        panel.apply_manual_selection({
            "current_channel": 44,
            "ranking": [
                dict(channel=36, score=0, neighbors=0),
                dict(channel=44, score=36, neighbors=3),
            ],
        })
        self.assertEqual(panel.pending_candidate, 36)
        self.assertTrue(panel.manual_select_requested)

    def test_manual_selection_keeps_current_best_channel(self):
        panel = NetworkPanel.__new__(NetworkPanel)
        panel.status = Mock()
        panel.append = Mock()
        panel.last_change = 0.0
        panel.pending_candidate = None
        panel.manual_select_requested = True
        panel.apply_manual_selection({
            "current_channel": 36,
            "ranking": [
                dict(channel=36, score=0, neighbors=0),
                dict(channel=44, score=20, neighbors=2),
            ],
        })
        self.assertIsNone(panel.pending_candidate)
        self.assertFalse(panel.manual_select_requested)

    def test_failed_baseline_never_changes_router(self):
        router = Mock()
        with patch("dual_dog.devices.wifi_optimizer.ping_fleet", return_value=[dict(ip="dog", received=0, sent=3)]):
            with self.assertRaises(RuntimeError):
                change_and_verify(router, {}, 149, ["dog"])
        router.set_channel.assert_not_called()

    def test_manual_change_ignores_known_offline_dog(self):
        router = Mock()
        snapshot = {"info": [{}, dict(channel="149", bandwidth="40", ssid="Router")]}
        router.wifi.side_effect = [
            {"info": [{}, dict(channel="36", bandwidth="40")]},
        ]
        baseline = [
            dict(ip="online", received=3, sent=3),
            dict(ip="offline", received=0, sent=3),
        ]
        good = [dict(ip="online", received=3, sent=3) for _ in range(3)]
        with patch("dual_dog.devices.wifi_optimizer.ping_fleet", side_effect=[baseline, good, good, good]), \
             patch("dual_dog.devices.wifi_optimizer.write_json"), patch("dual_dog.devices.wifi_optimizer.time.sleep"):
            change_and_verify(
                router,
                snapshot,
                36,
                ["online", "offline"],
                require_all_reachable=False,
            )
        router.set_channel.assert_called_once_with(snapshot, 36, 40)

    def test_bad_post_change_reception_rolls_back_and_verifies(self):
        router = Mock()
        snapshot = {"info": [{}, dict(channel="36", bandwidth="40", ssid="Router")]}
        router.wifi.side_effect = [
            {"info": [{}, dict(channel="149", bandwidth="40")]},
            {"info": [{}, dict(channel="36", bandwidth="40")]},
        ]
        good = [dict(ip="dog", received=3, sent=3)]
        bad = [dict(ip="dog", received=0, sent=3)]
        with patch("dual_dog.devices.wifi_optimizer.ping_fleet", side_effect=[good, bad, bad, bad]), \
             patch("dual_dog.devices.wifi_optimizer.write_json"), patch("dual_dog.devices.wifi_optimizer.time.sleep"):
            change_and_verify(router, snapshot, 149, ["dog"])
        self.assertEqual([call.args[1:] for call in router.set_channel.call_args_list], [(149, 40), ("36", "40")])

    def test_any_complete_round_for_one_dog_is_unsafe(self):
        baseline = [dict(ip="dog", received=3, sent=3)]
        observations = [
            dict(ip="dog", received=0, sent=3),
            dict(ip="dog", received=3, sent=3),
            dict(ip="dog", received=3, sent=3),
        ]
        self.assertTrue(reception_unsafe(baseline, observations, ["dog"]))

    def test_ten_percent_loss_is_rejected(self):
        baseline = [dict(ip="dog", received=3, sent=3)]
        observations = [
            dict(ip="dog", received=3, sent=3),
            dict(ip="dog", received=3, sent=3),
            dict(ip="dog", received=2, sent=3),
        ]
        self.assertTrue(reception_unsafe(baseline, observations, ["dog"]))

    def test_clean_post_change_reception_is_accepted(self):
        baseline = [dict(ip="dog", received=3, sent=3)]
        observations = [dict(ip="dog", received=3, sent=3) for _ in range(3)]
        self.assertFalse(reception_unsafe(baseline, observations, ["dog"]))


if __name__ == "__main__":
    unittest.main()
