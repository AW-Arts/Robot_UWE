# Electrical LED wiring and pin-out diagrams

This guide shows exactly how to wire the six prototype status LEDs to the SSC-32(U) without additional GPIO or power hardware. It complements the behaviour and channel map in `docs/status_led_plan.md` with per-channel pin-outs and harness diagrams you can hand to a technician.

## Fixed channel map (prototype)

| LED | SSC-32 channel |
| --- | --- |
| Red | 8 |
| Amber | 9 |
| Green | 10 |
| Blue | 11 |
| Yellow | 12 |
| White | 13 |

## Wiring rules (carryover from the plan)

- Drive each LED from its SSC-32 signal pin through a **dedicated 220–470 Ω resistor**.
- LED cathodes go to **ground**. Never tie LEDs between two signal pins.
- Leave the servo **V+ pin unused**; LEDs are fed from the signal line only.
- Do **not** share resistors between LEDs.
- Channels 0–7 are reserved for servos.

## Pin-out reference

Each three-pin servo header on the SSC-32(U) is laid out as **Signal (S) – V+ – Ground (G)** when read top-to-bottom with the board silkscreen upright. Only the signal and ground pins are used for LEDs.

```
Channel header (top → bottom):
 S  Signal ──[220–470 Ω]──▶|── LED anode
 +  V+     (leave empty)
 G  Ground ────────────────┴── LED cathode / shared ground bus
```

## Single-channel wiring diagram

```
SSC-32 SIGNAL (CHx) ──[220–470 Ω]──▶|── LED ── GND
```

## Multi-LED harness diagram (channels 8–13)

```
        ┌───────────────────────────────────────────────────────────────┐
        │            SSC-32(U) servo headers (channels 8–13)            │
        │                                                               │
        │  #8   #9   #10  #11  #12  #13                                 │
Signal: │  S    S    S    S    S    S   ──[220–470 Ω]──▶|── LED anodes   │
        │                                                               │
 V+:    │  +    +    +    +    +    +   (leave empty)                   │
        │                                                               │
Ground: │  G    G    G    G    G    G ────────────────┬─────────────────┘
        │                                            (shared ground bus to all LED cathodes)
        └───────────────────────────────────────────────────────────────┘

Legend:
- #8 = Red, #9 = Amber, #10 = Green, #11 = Blue, #12 = Yellow, #13 = White
- Resistors live on the signal leg for each LED. Ground can be bussed.
```

## Installation checklist

- [ ] Confirm the silk labels for **S / + / G** on channels 8–13 before soldering.
- [ ] Terminate the **ground bus first**, then run individual signal+resistor leads to each LED anode.
- [ ] Keep resistors close to the controller header to avoid loose floating leads.
- [ ] Verify polarity (▶|) before powering on; swap if an LED stays dark at 2000 µs.
- [ ] Re-read the prototype limitation in `docs/status_led_plan.md` and keep this wiring to prototype use only.
