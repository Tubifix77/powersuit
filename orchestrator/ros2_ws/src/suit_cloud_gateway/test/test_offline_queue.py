"""Native tests for suit_cloud_gateway.offline_queue — ROS-free."""

import pytest

from suit_cloud_gateway.offline_queue import DEFAULT_MAXLEN, OfflineQueue


def test_default_bound_is_512():
    assert DEFAULT_MAXLEN == 512
    q = OfflineQueue()
    assert q.maxlen == 512


def test_bounds_drop_oldest():
    q = OfflineQueue(maxlen=512)
    for i in range(600):
        q.put(i)
    assert len(q) == 512
    assert q.dropped == 88
    items = q.drain()
    assert items[0] == 88          # oldest surviving
    assert items[-1] == 599        # newest


def test_drain_order_fifo_and_clears():
    q = OfflineQueue(maxlen=4)
    for x in ("a", "b", "c"):
        q.put(x)
    assert q.drain() == ["a", "b", "c"]
    assert len(q) == 0
    assert q.drain() == []


def test_wrap_keeps_newest():
    q = OfflineQueue(maxlen=3)
    for i in range(5):
        q.put(i)
    assert q.peek_all() == [2, 3, 4]
    assert q.drain() == [2, 3, 4]
    assert q.dropped == 2


def test_put_after_drain_starts_fresh():
    q = OfflineQueue(maxlen=2)
    q.put(1)
    q.put(2)
    q.put(3)
    assert q.drain() == [2, 3]
    q.put(9)
    assert q.drain() == [9]


def test_invalid_maxlen():
    with pytest.raises(ValueError):
        OfflineQueue(maxlen=0)
