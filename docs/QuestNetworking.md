# Quest USB and Wi-Fi streaming

USB remains the default. Both transports use the same outbound TCP protocols:
pose on 5005 and anchor alignment on 5006. The computer receives connections;
no listening port, multicast or broadcast is added to the headset.

## Select transport

Run from the repository root with the Quest connected to ADB for setup:

```bash
# Use your receiver computer's LAN address, not the Quest address.
python3 scripts/quest-network.py wifi --host 192.168.1.158 --restart

# Return to USB; this also restores both adb reverse mappings.
python3 scripts/quest-network.py usb --restart
```

The example IP was the development computer's address during validation; replace
it if the computer/network changes. `--serial` selects a device when necessary.
`--dry-run` prints configuration without touching the device. `--port` changes
the pose destination port; alignment always uses port+1.

Settings are stored in the app's external `Saved/QuestNetwork.ini`:

```ini
[APLC.Network]
Transport=WiFi
Host=192.168.1.158
PosePort=5005
```

Read once at BeginPlay; restart after editing. `Transport=USB` uses 127.0.0.1
regardless of Host. Missing settings preserve the existing Blueprint component
configuration (normally USB). Invalid settings are logged and both TCP senders
are prevented from connecting. No Blueprint assets or wire formats change.
ADB is needed to configure this version, but can be disconnected for Wi-Fi use.
There is no automatic failover or simultaneous duplicate stream in this version.

## Receiver and Docker ports

A receiver must bind `0.0.0.0` inside Docker (or the host LAN interface for a
native receiver), not only loopback. Both Wi-Fi and USB can use the same receiver.
Only one receiver/container can own each host port at a time.

The RViz launcher has an optional receiver-port mapping:

```bash
bash scripts/docker-rviz.sh --quest-network
```

This publishes TCP 5005/5006 on all host IPv4 interfaces via
`docker/compose.quest-network.yml`; normal static RViz startup is unchanged.
The wrapper passes `--service-ports` because Compose `run` normally ignores
published ports. Optional environment settings: `APLC_QUEST_BIND_IP`,
`APLC_QUEST_POSE_PORT`, `APLC_QUEST_ALIGNMENT_PORT` change host bindings; container
ports remain 5005/5006. Binding 127.0.0.1 restricts access to USB/local use.
Host firewall policy and Wi-Fi client isolation must allow the connection; no
host firewall configuration is changed by these scripts.

Publishing Docker ports alone does not create a receiver or make RViz consume
TCP. The live ROS bridge is now available with `bash scripts/docker-rviz.sh --live`;
see [Live Quest](LiveQuest.md).
To check reception now without ROS, use the temporary diagnostic receiver:

```bash
# Native receiver (on the computer):
python3 scripts/quest-check-streams.py --bind 0.0.0.0 \
  --seconds 60 --output /tmp/quest-network-check

# Or inside the existing Docker image:
docker run --rm --user "$(id -u):$(id -g)" \
  -p 5005:5005 -p 5006:5006 \
  -v "$PWD:/ws" -w /ws aplc-dev:jazzy \
  python3 scripts/quest-check-streams.py --bind 0.0.0.0 \
  --seconds 60 --output /ws/log/quest-network-check
```

The diagnostic skips occupied ports and exits after the specified duration. It
records counts, alignment status and the latest messages; it does not publish
ROS. With Wi-Fi selected, neither stream uses `adb reverse`.

## Validation — November 4, 2026

Linux Editor and Android ARM64 build/cook/package passed; APK installed.
Docker Compose resolved explicit TCP mappings for both ports. Host setup helper
validation, shell syntax, and git whitespace checks passed. An initial UE config
parser signature mismatch was corrected before the successful build.

Wi-Fi test: Quest 192.168.1.157 -> computer 192.168.1.158 -> Docker receiver.
Both adb reverse rules were removed during this test: 1,176 pose messages and
61 alignment messages, zero invalid frames; alignment reached valid with all
142 anchors. USB was then restored and retested: 741 pose messages and
38 alignment messages, zero invalid frames; alignment again reached
valid with all 142 anchors. Temporary receivers exited and released the ports.

Device is left configured for USB with both reverse rules restored. No firewall
rules were manually changed. The ROS live bridge was added subsequently; see [Live Quest](LiveQuest.md).
