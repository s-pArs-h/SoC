"""Tests for the host side (pytest host/). The RTL is tested in tb/."""
import random
import struct

import pytest

import kmeans_host
import kmeans_ref as ref
import protocol as p
from fake_board import FakeBoard


def test_crc_check_value():
    # standard check value of CRC-16/CCITT-FALSE
    assert p.crc16(b"123456789") == 0x29B1


def test_request_frame_layout():
    f = p.request(p.CMD_LABELS, p.labels_payload(5, 7))
    assert f[0] == p.SYNC_REQ and f[1] == p.CMD_LABELS
    assert struct.unpack_from("<H", f, 2)[0] == 4
    assert struct.unpack_from("<H", f, len(f) - 2)[0] == p.crc16(f[1:-2])


def test_response_crc_is_checked():
    board = FakeBoard()
    board.write(p.request(p.CMD_INFO))
    frame = bytearray(board.read(1000))
    assert p.parse_response(bytes(frame)).status == p.ST_OK
    frame[6] ^= 1
    with pytest.raises(p.ProtocolError):
        p.parse_response(bytes(frame))


def test_round_div_rounds_half_away_from_zero():
    assert [ref.round_div(s, 2) for s in (3, -3, 1, -1, 4)] == [2, -2, 1, -1, 2]


def test_lloyd_recovers_separated_clusters():
    rng = random.Random(3)
    centres = [(-20000, -20000), (20000, -20000), (-20000, 20000), (20000, 20000)]
    pts = [(cx + rng.randint(-500, 500), cy + rng.randint(-500, 500))
           for cx, cy in centres for _ in range(100)]
    r = ref.lloyd(pts, [pts[0], pts[100], pts[200], pts[300]], 50)
    assert r["converged"] and r["counts"] == [100, 100, 100, 100]


def test_random_start_can_get_stuck():
    # Why the host uses k-means++: two nearby clusters share one centroid
    # while a far cluster is split between two, and Lloyd's algorithm cannot
    # leave that local minimum. k-means++ spreads the start out (it lowers
    # the chance of this, it does not remove it).
    rng = random.Random(3)
    centres = [(-20000, 0), (-14000, 0), (20000, -15000), (20000, 15000)]
    pts = [(cx + rng.randint(-1500, 1500), cy + rng.randint(-1500, 1500))
           for cx, cy in centres for _ in range(100)]
    stuck = ref.lloyd(pts, [pts[0], pts[200], pts[201], pts[300]], 50)
    assert stuck["converged"] and stuck["counts"] == [200, 60, 40, 100]
    good = ref.lloyd(pts, ref.kmeans_pp(pts, 4, random.Random(1)), 50)
    assert sorted(good["counts"]) == [100, 100, 100, 100]


def test_host_end_to_end_with_fake_board(capsys):
    assert kmeans_host.main(["--fake", "--points", "500", "--seed", "4"]) == 0
    out = capsys.readouterr().out
    assert out.count("matches the reference") == 2


def test_csv_scaling(tmp_path):
    f = tmp_path / "d.csv"
    f.write_text("x,y\n0.5,1.0\n-0.5,-1.0\n0.25,0.0\n0,0\n")
    pts, scaled = kmeans_host.read_csv(str(f))
    assert scaled is not None
    assert max(max(abs(x), abs(y)) for x, y in pts) == 30000
    assert kmeans_host.main(["--fake", "--csv", str(f)]) == 0
