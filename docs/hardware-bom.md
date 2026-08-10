# Hardware bill of materials

What the suit costs, derived from the actual design: `docs/network-map.md` §1 for
the node list, and each app's `board_*.h` for what hangs off it.

**Read the caveats before quoting any of this at anyone.**

- Prices are **indicative, not quotes**. AliExpress listings could not be fetched
  directly (the site returns 502 to automated requests), so most figures here are
  typical-street-price ranges rather than a live basket. Two are sourced exactly
  and marked as such.
- Marketplace prices move constantly and vary several-fold between sellers for
  what looks like the same part.
- Nothing here has been bought or verified. This is a planning document.
- **Actuators dominate everything.** Read §4 before budgeting.

---

## 1. Three tiers, because "a powersuit" spans two orders of magnitude

| Tier | What it is | Actuators | Rough total |
|------|-----------|-----------|-------------|
| **A — Bench** | Two boards proving the network and safety logic. Already ordered. | none | **~€60** |
| **B — Cosplay** | Full nine-node electronics, articulated shell, non-load-bearing. Lights, sound, HUD, servo-driven panels. | hobby servos | **~€700–1,100** |
| **C — Spec** | The torque figures actually written in `board_limb.h`. A machine that moves a human. | 8 robot joints | **~€12,000–35,000+** |

Tier B is the one worth costing carefully — it is a real project someone can
finish. Tier C is costed here mainly so the number is on the record.

---

## 2. Electronics common to all tiers (the nine nodes)

This is the part this repository is actually about, and it is the cheap part.

| Node | Item | Qty | Unit | Subtotal | Note |
|------|------|-----|------|----------|------|
| 1–4, 5, 6 | ESP32-S3-DevKitC-1 N16R8 | 6 | ~€8 | €48 | eBay listing seen at **US $7.41**; AliExpress typically similar or lower |
| 7 | ESP32-P4 dev board | 1 | ~€20 | €20 | Newer part, thinner supply, wider price spread |
| 1–7 | SN65HVD230 CAN transceiver | 7 | ~€4 | €28 | One per node; only the two bus ends keep their 120 Ω |
| 1–5 | MPU-6050 IMU (or MPU-6500) | 5 | ~€3 | €15 | Four limbs plus the torso IMU on node 5 |
| 6 | INMP441 I2S MEMS microphone | 1 | ~€4 | €4 | |
| 6 | MAX98357A I2S amp + small speaker | 1 | ~€6 | €6 | |
| 6 | ST7789 240×240 SPI display | 1 | ~€7 | €7 | Stand-in for the spec's twin micro-OLEDs |
| 7 | WS2812 24-pixel ring | 1 | ~€5 | €5 | Arc reactor |
| 8 | Raspberry Pi 5 (8 GB) + PSU + SD | 1 | ~€110 | €110 | |
| all | Wiring, connectors, JST/XT60, heatshrink | — | — | ~€60 | Harness for a whole suit adds up |
| | **Electronics subtotal** | | | **~€300** | |

Node 9 is cloud-side: **€0** with the bundled mock engine, or roughly **€5–30/month**
for a small VPS, or GPU-hourly if you point it at real inference.

---

## 3. Tier B — cosplay build (non-load-bearing)

Adds to §2. The shell moves panels and looks alive; it does not carry you.

| Item | Qty | Unit | Subtotal |
|------|-----|------|----------|
| MG996R / DS3218 servo (flight surfaces, node 5) | 12 | ~€6 | €72 |
| MG996R servo (limb articulation, cosmetic) | 8 | ~€6 | €48 |
| MPXV7002DP differential pressure sensor (pitot) | 1 | ~€12 | €12 |
| 6S LiPo 5000 mAh + charger | 1 | ~€70 | €70 |
| 5 V / 6 V BEC regulators, 10 A | 3 | ~€8 | €24 |
| Fuses, main switch, XT60 distribution | — | — | €25 |
| Shell: 3D printing filament / EVA foam | — | — | €150–400 |
| **Tier B subtotal (excl. §2)** | | | **~€400–650** |
| **Tier B total** | | | **~€700–1,100** |

Servos on this scale draw real current — budget the BEC and battery properly, and
keep servo power off the ESP32 rails entirely.

---

## 4. Tier C — the suit as `board_limb.h` actually specifies it

This is where the honesty matters. The joint table specifies:

| Joint | Torque limit | ×2 limbs |
|-------|--------------|----------|
| Elbow | 45 N·m | 2 |
| Wrist | 18 N·m | 2 |
| Hip | 140 N·m | 2 |
| Knee | 120 N·m | 2 |

For scale, a **CubeMars AK80-9 is $479.90 for 9 N·m rated** ([store.cubemars.com](https://store.cubemars.com/products/ak80-9)).
That is an integrated unit — frameless motor, planetary gearbox, encoder and FOC
driver in one housing — and it is one fifth of the torque the *wrist* wants.

Integrated joint modules spanning this range exist (rated torques from roughly
2 N·m to 586 N·m, with harmonic reducers and absolute encoders — see
[CubeMars AKE series](https://www.cubemars.com/categorys/ake-series-robotic-actuator)
and the Made-in-China exoskeleton-actuator vendors), but they are
quote-on-request rather than cart-and-checkout.

Realistic planning figures, **estimates not quotes**:

| Item | Qty | Unit estimate | Subtotal |
|------|-----|---------------|----------|
| 18 N·m joint module (wrist) | 2 | €600–1,200 | €1,200–2,400 |
| 45 N·m joint module (elbow) | 2 | €900–2,000 | €1,800–4,000 |
| 120 N·m joint module (knee) | 2 | €1,800–4,500 | €3,600–9,000 |
| 140 N·m joint module (hip) | 2 | €2,000–5,000 | €4,000–10,000 |
| AS5047P absolute encoders (if not integrated) | 8 | €12 | €96 |
| Load cells + HX711 (force sensing) | 16 | €8 | €128 |
| BMS power stage: FETs, gate driver, shunt, **analog comparator + SR latch** | 1 | — | €150–300 |
| 12S LiPo/Li-ion pack ~50 V (matches `board_hub.h` 5020 cV) + charger | 1 | €400–900 | €400–900 |
| Structural frame, bearings, load paths | — | — | €1,000–5,000 |
| **Tier C total** | | | **~€12,000–35,000+** |

**Roughly 80–90% of that is eight actuators.** Every other decision in this
repository — the CAN topology, the protocol, the safety machine — is rounding
error against the joint modules. That is worth knowing before anyone treats the
firmware as the hard part.

The comparator and latch in the BMS row are not optional and not substitutable
by firmware: see `docs/safety.md` §4.

---

## 5. What to actually buy, in order

1. **Tier A bench (~€60)** — `docs/bringup.md` Appendix A. Proves the network and
   the dead-man switch. Already ordered.
2. **Add the §2 electronics incrementally** as each node becomes interesting.
   Every node except the hub is under €20 of parts.
3. **Decide Tier B or C before buying anything mechanical.** They share the
   electronics in §2 and share nothing else.

If the answer is cosplay — and `docs/safety.md` was written for a machine that can
hurt someone, so this is a real fork — then Tier C never happens and the suit
costs about a thousand euros, most of it shell and servos.

---

## Sources

- [CubeMars AK80-9 store listing](https://store.cubemars.com/products/ak80-9) — $479.90, 9 N·m rated (exact, fetched)
- [CubeMars AKE series robotic actuators](https://www.cubemars.com/categorys/ake-series-robotic-actuator) — integrated joint module range
- [ESP32-S3-DevKitC-1 N16R8, eBay listing](https://www.ebay.com/itm/355163211698) — US $7.41 (exact, from search results)
- [ESP32-S3-DevKitC-1 N16R8 on AliExpress](https://www.aliexpress.com/item/1005006240070551.html) — price not retrievable automatically
- [Exoskeleton BLDC joint modules, Made-in-China](https://metonec.en.made-in-china.com/product/gJwYiQOHvxpI/China-Robot-Motors-Exoskeleton-Robot-BLDC-Motor-Suit-Harmonic-Drive-Robotic-Joint-Module-Motor.html) — quote-on-request vendor

Everything not marked *exact* is a planning estimate. Re-check before ordering.
