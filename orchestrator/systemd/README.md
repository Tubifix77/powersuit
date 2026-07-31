# Deploying Node 8

`../scripts/install_pi.sh` does the whole provisioning run on a Raspberry Pi 5
(Ubuntu 24.04 arm64) and installs the unit in this directory. What follows is
what the script does not, and cannot, decide for you.

## Before first run

**Enable SPI.** Add `dtparam=spi=on` to `/boot/firmware/config.txt` and reboot.
The bridge opens `/dev/spidev0.0` at 20 MHz (`docs/network-map.md` §12.1).

**Set the link token.** `/etc/powersuit/env` is created with a placeholder:

```bash
sudo install -m 0600 /dev/stdin /etc/powersuit/env <<'EOF'
POWERSUIT_LINK_TOKEN=<the real token>
EOF
```

Node 9 rejects an unknown token with WebSocket close 4001, and the gateway backs
off exponentially rather than hammering it.

**Wire DATA_READY** if you want interrupt-driven SPI service rather than 1 kHz
polling. Set `gpiochip` and `data_ready_line` in
`suit_bringup/config/params.yaml`; leaving `gpiochip` empty selects polling,
which works but burns a little more CPU.

## Real-time behaviour

The unit grants `CAP_SYS_NICE` so the bridge's transport thread can put itself
on `SCHED_FIFO` without the whole ROS graph running as root. If you see
`SCHED_FIFO refused (EPERM)` in the log, that capability did not survive — the
bridge keeps working at normal priority, but the 1 kHz SPI cadence will jitter
under load.

For a suit that actually flies, use a `PREEMPT_RT` kernel and isolate a core:
add `isolcpus=3 nohz_full=3 rcu_nocbs=3` to the kernel command line and pin the
bridge to CPU 3. Verify with `cyclictest -p80 -t1 -n` before trusting it.

## Operating

```bash
systemctl enable --now powersuit     # start at boot
journalctl -u powersuit -f           # follow
systemctl stop powersuit             # heartbeats stop -> the suit goes limp
```

That last point is the important one: stopping the service is a safe action.
The limbs see the heartbeat stop and drop into Passive Compliance within 50 ms
(`docs/safety.md` §2). There is no state in which killing Node 8 leaves an
actuator energised.

## Black-box recovery

The ring buffer lives in `/var/lib/powersuit`. On an e-stop the recorder freezes
and writes `blackbox-<iso8601>.bin` alongside it. That directory also holds
`estop_counter`, the monotonic counter that makes a replayed `CLEAR_ESTOP`
frame useless — **do not delete it**. Losing it means the next clear you issue
may be rejected as stale until the counter catches up.
