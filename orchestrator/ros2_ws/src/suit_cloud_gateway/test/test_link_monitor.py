"""Native tests for suit_cloud_gateway.link_monitor — ROS-free, injected clock."""

import pytest

from suit_cloud_gateway.link_monitor import Bearer, Decision, LinkMonitor


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


BEARERS = [
    Bearer("5g", "wss://a/link", 1, 150.0),
    Bearer("wifi", "wss://b/link", 2, 80.0),
    Bearer("sat", "wss://c/link", 3, 1200.0),
]


@pytest.fixture()
def clock():
    return FakeClock()


@pytest.fixture()
def mon(clock):
    return LinkMonitor(BEARERS, clock=clock)


def test_scoring_healthy_requires_all_three(mon):
    mon.update("wifi", connected=True, rtt_ms=40.0, loss_pct=0.1)
    assert mon.is_healthy("wifi")
    mon.update("wifi", connected=True, rtt_ms=90.0, loss_pct=0.1)   # over budget
    assert not mon.is_healthy("wifi")
    mon.update("wifi", connected=True, rtt_ms=40.0, loss_pct=5.0)   # loss at limit
    assert not mon.is_healthy("wifi")
    mon.update("wifi", connected=False, rtt_ms=40.0, loss_pct=0.0)  # not connected
    assert not mon.is_healthy("wifi")
    mon.update("sat", connected=True, rtt_ms=900.0, loss_pct=1.0)   # sat budget 1200
    assert mon.is_healthy("sat")


def test_initial_attach_prefers_priority(mon):
    mon.update("wifi", connected=True, rtt_ms=30.0)
    mon.update("5g", connected=True, rtt_ms=50.0)
    d = mon.evaluate()
    assert d.switch_to == "5g" and d.make_before_break is False
    mon.commit("5g")
    assert mon.active == "5g"
    assert mon.switches == 0  # first attach is not a switch


def test_unhealthy_grace_3s_then_switch(mon, clock):
    mon.update("5g", connected=True, rtt_ms=50.0)
    mon.update("wifi", connected=True, rtt_ms=30.0)
    mon.commit("5g")

    clock.advance(100.0)
    mon.update("5g", connected=False)         # active dies at t=100
    clock.advance(2.9)
    d = mon.evaluate()                        # t=102.9: inside 3 s grace
    assert d.switch_to is None
    clock.advance(0.2)
    d = mon.evaluate()                        # t=103.1: grace expired
    assert d.switch_to == "wifi"
    assert d.make_before_break is True
    mon.commit("wifi")
    assert mon.switches == 1


def test_higher_priority_needs_10s_continuous(mon, clock):
    mon.update("sat", connected=True, rtt_ms=800.0)
    mon.commit("sat")

    mon.update("5g", connected=True, rtt_ms=40.0)   # 5g comes up at t=0
    clock.advance(9.9)
    mon.update("sat", connected=True, rtt_ms=800.0)  # keep active healthy
    assert mon.evaluate().switch_to is None          # 9.9 s < 10 s
    clock.advance(0.2)
    d = mon.evaluate()                               # 10.1 s continuous
    assert d.switch_to == "5g" and d.make_before_break is True


def test_flapping_bearer_resets_continuity(mon, clock):
    mon.update("sat", connected=True, rtt_ms=800.0)
    mon.commit("sat")

    mon.update("5g", connected=True, rtt_ms=40.0)
    clock.advance(6.0)
    mon.update("5g", connected=False)                # blip at t=6 resets the timer
    clock.advance(1.0)
    mon.update("5g", connected=True, rtt_ms=40.0)    # healthy again at t=7
    clock.advance(9.0)
    assert mon.evaluate().switch_to is None          # only 9 s since t=7
    clock.advance(1.5)
    assert mon.evaluate().switch_to == "5g"          # 10.5 s continuous


def test_no_alternative_stays_put(mon, clock):
    mon.update("wifi", connected=True, rtt_ms=30.0)
    mon.commit("wifi")
    mon.update("wifi", connected=False)
    clock.advance(5.0)
    d = mon.evaluate()
    assert d.switch_to is None
    assert "no alternative" in d.reason


def test_lower_priority_healthy_never_preempts(mon, clock):
    mon.update("5g", connected=True, rtt_ms=40.0)
    mon.commit("5g")
    mon.update("sat", connected=True, rtt_ms=500.0)
    clock.advance(60.0)
    mon.update("5g", connected=True, rtt_ms=40.0)
    assert mon.evaluate().switch_to is None


def test_switch_counter_counts_commits(mon):
    mon.update("5g", connected=True, rtt_ms=40.0)
    mon.update("wifi", connected=True, rtt_ms=30.0)
    mon.commit("5g")
    mon.commit("wifi")
    mon.commit("wifi")   # re-commit same bearer: not a switch
    mon.commit("5g")
    assert mon.switches == 2
