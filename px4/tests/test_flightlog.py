"""Reader checks for the SITL logs (spec 9.3, 9.5) and the consistency of the expected visit.
Run: px4/py.sh tests/test_flightlog.py"""

import struct
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import expected  # noqa: E402
import flightlog  # noqa: E402


def header(magic: bytes, record: int, source_hash: int = 0) -> bytes:
    return struct.pack("<8sHHIIIQ", magic, 1, 0, 32, record, 0, source_hash)


def lane(**kw) -> bytes:
    f = dict(t0=1000, t1=150000, lane=3, order=0, payload=5.0, energy=3e5, s0=0.6, s1=0.5,
             drone=0, a=0, b=4, source=1)
    f.update(kw)
    return struct.pack("<QQIIffffHHHBx", f["t0"], f["t1"], f["lane"], f["order"], f["payload"],
                       f["energy"], f["s0"], f["s1"], f["drone"], f["a"], f["b"], f["source"])


class Files(unittest.TestCase):
    def write(self, data: bytes) -> Path:
        d = Path(tempfile.mkdtemp())
        p = d / "f.bin"
        p.write_bytes(data)
        return p

    def test_lane_round_trip(self):
        lanes = flightlog.read_lanes(self.write(header(b"DDFLOG\0\0", 48) + lane()))
        self.assertEqual(len(lanes), 1)
        self.assertEqual((lanes[0].lane_id, lanes[0].to_pad, lanes[0].source), (3, 4, 1))
        self.assertAlmostEqual(lanes[0].payload_kg, 5.0)

    def test_lane_rejects(self):
        cases = {
            "magic": header(b"DDEVLOG\0", 48) + lane(),
            "hash": header(b"DDFLOG\0\0", 48, source_hash=7) + lane(),
            "partial": header(b"DDFLOG\0\0", 48) + lane()[:40],
            "time order": header(b"DDFLOG\0\0", 48) + lane(t0=5, t1=4),
            "source DES": header(b"DDFLOG\0\0", 48) + lane(source=0),
            "soc": header(b"DDFLOG\0\0", 48) + lane(s1=1.5),
        }
        for name, data in cases.items():
            with self.subTest(name), self.assertRaises(flightlog.LogError):
                flightlog.read_lanes(self.write(data))

    def test_sample_layout(self):
        rec = np.zeros(1, dtype=flightlog.SAMPLE_DTYPE)
        rec["t_us"] = 123
        rec["motor_radps"] = [1, 2, 3, 4]
        rec["drone_mode"] = 3
        raw = rec.tobytes()
        # Offsets of spec 9.5.
        self.assertEqual(struct.unpack_from("<Q", raw, 0)[0], 123)
        self.assertEqual(struct.unpack_from("<4f", raw, 48), (1.0, 2.0, 3.0, 4.0))
        self.assertEqual(raw[88], 3)
        s = flightlog.read_samples(self.write(header(b"DDSAMPL\0", 96) + raw))
        self.assertEqual(int(s["t_us"][0]), 123)


class SendOrder(unittest.TestCase):
    def test_same_tick_messages_of_one_sender_follow_their_sequence(self):
        # Received swapped (DDS topics are independent), sent in sequence order.
        def msg(name, sender, pad, seq, t):
            return {"name": name, "sender": sender, "pad_id": pad, "seq": seq, "t_us": t}
        received = [msg("LATCH_REPORT", "DRONE", 4, 9, 100), msg("SERVICE_COMPLETE", "PAD", 4, 7, 200),
                    msg("POD_LATCHED", "PAD", 4, 6, 200), msg("MISSION_REQUEST", "DRONE", 4, 10, 200)]
        names = [m["name"] for m in flightlog.send_order(received)]
        self.assertEqual(names, ["LATCH_REPORT", "POD_LATCHED", "SERVICE_COMPLETE", "MISSION_REQUEST"])


class ExpectedVisit(unittest.TestCase):
    def chain(self, rows):
        for (_, _, to), (_, frm, _) in zip(rows, rows[1:]):
            self.assertEqual(to, frm)

    def test_state_sequences_chain(self):
        self.chain(expected.VISIT_A)
        self.chain(expected.VISIT_B)
        self.chain(expected.DRONE)

    def test_messages_alternate_sensibly(self):
        names = [n for _, n, _, _ in expected.MESSAGES]
        self.assertEqual(names.count("LANDED"), 2)
        self.assertLess(names.index("RELEASE_CMD"), names.index("RELEASE_REPORT"))
        self.assertLess(names.index("POD_DELIVERED"), names.index("HOP_CMD"))


if __name__ == "__main__":
    unittest.main()
