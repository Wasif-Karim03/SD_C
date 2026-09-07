# robocar — Camera & Perception Reference

**Teaching the car to see, with two cheap cameras.**

What each camera should actually do, which 2025-era models run on the Jetson Orin Nano, how vision fuses with your LiDAR, GPS and wheel odometry, and the honest limits of a 640×480 USB webcam. A perception plan built around *your* hardware and your goal: drive A→B faster, safer, and smoother — by itself.

| | |
|---|---|
| **Cameras** | 2× icSpring USB · front + rear |
| **Format** | YUYV 640×480 · ~18–22 fps |
| **Compute** | Jetson Orin Nano Super (MAXN_SUPER, ~67 TOPS) |
| **Other sensors** | RPLIDAR C1 · GPS (M8N) + IST8310 compass · VESC odom |
| **Target** | Point-to-point autonomous driving |
| **Compiled** | 2026-08-08 |

---

## Contents

1. [The short version](#1--the-short-version)
2. [Your camera reality](#2--your-camera-reality)
3. [Camera vs LiDAR vs GPS vs odometry](#3--camera-vs-lidar-vs-gps-vs-odometry)
4. [Recommended architecture](#4--recommended-architecture)
5. [Which stack — and why nav2](#5--which-stack--and-why-nav2)
6. [Front camera jobs](#6--front-camera-jobs)
7. [Rear camera jobs](#7--rear-camera-jobs)
8. [Depth & free-space — the deep dive](#8--depth--free-space--the-deep-dive)
9. [Object detection — the deep dive](#9--object-detection--the-deep-dive)
10. [Lane & drivable area — the deep dive](#10--lane--drivable-area--the-deep-dive)
11. [Localization & visual odometry](#11--localization--visual-odometry)
12. [Fusing it all together](#12--fusing-it-all-together)
13. [The compute budget](#13--the-compute-budget)
14. [Calibration, timing & latency](#14--calibration-timing--latency)
15. [What a proper self-driving car must see](#15--what-a-proper-self-driving-car-must-see)
16. [Safety architecture](#16--safety-architecture)
17. [Faster & smoother](#17--faster--smoother)
18. [Model shortlist](#18--model-shortlist)
19. [Roadmap](#19--roadmap)
20. [Sources](#20--sources)

---

## 1 · The short version

**Cameras give meaning; LiDAR gives geometry.**

Your 2D LiDAR already answers *"is something there, and how far?"* reliably, in a flat 360° ring at one fixed height. It cannot tell a person from a pole, cannot see a curb, a pothole, a low box, an overhanging shelf, a closed glass door, a stop sign, or a painted lane. **That gap is exactly the camera's job.** The camera is not a better rangefinder — it is the sensor that adds **semantics** (what things are), **3D free-space off the scan plane** (low and high obstacles), and **road/path cues** (lanes, drivable surface, signs, lights).

- **Front camera = the workhorse.** Metric depth → free-space, YOLO detection → people/vehicles/signs, drivable-area segmentation → path following. Three models on one image stream.
- **Rear camera = safety & recovery.** Reverse without blind backing, power the dead-end recovery routine, watch for something approaching from behind, and aid visual odometry.
- **The stack = nav2 + a reflex.** ROS 2 nav2 for real A→B planning; a low-latency depth "reflex" layer for emergency stop. Camera feeds the costmap; LiDAR anchors geometry.

> **The single most important realization.** On this rig the **camera sensor is the bottleneck, not the GPU**. Your webcams cap at ~20 fps (YUYV, no MJPG), while a YOLO11n runs in ~4 ms and depth in ~24 ms on the Orin. That means you have compute headroom to run *several* perception models on every frame — the design question is what to *do* with vision, not whether the board can keep up.

---

## 2 · Your camera reality

**What these two webcams can and can't do.**

Before any model choice, be honest about the sensor. Your hardware reference already flagged the camera as the perception bottleneck; here is exactly why, and what it means for the plan.

| Property | Your cameras | Why it matters for driving |
|---|---|---|
| Format / rate | YUYV 640×480, ~18–22 fps | No MJPG → raw frames eat USB bandwidth and cap FPS. At 2 m/s, 20 fps = a new look every 10 cm. Fine for slow; thin for fast. |
| Shutter | Rolling (assumed) | Fast motion smears & skews the image → depth and detection degrade exactly when you drive quickly. Motion blur is the enemy of "faster". |
| Stereo? | No — two separate views | Front + rear ≠ a stereo pair. No triangulated depth. All depth is *learned monocular* (scale-ambiguous). |
| Field of view | ~60–70° typical webcam | Narrow. Misses obstacles at the side during turns. LiDAR's 360° covers this; lean on it laterally. |
| Low light / glare | Weak (small sensor) | Auto-exposure already dragged you 15→9 fps. Night, sun glare, headlights = degraded vision. Plan for graceful fallback to LiDAR. |
| USB bandwidth | Both on Realtek hubs | Two raw YUYV streams at full rate can saturate USB 2.0. Run the idle-direction camera at low rate; only the active-direction camera at full rate. |

> **The highest-leverage hardware upgrade.** If you want "faster and smoother" to actually work, the biggest single win is a better camera: a **CSI/MIPI camera** (e.g. IMX219 / IMX477) plugs into the Jetson's camera port instead of USB — it frees USB bandwidth, uses the hardware ISP, and delivers **higher fps with less latency**. A **global-shutter** module additionally kills motion blur/skew at speed. Everything below works on your current webcams, but this is the ceiling-raiser.

---

## 3 · Camera vs LiDAR vs GPS vs odometry

**Who is responsible for what.**

Good autonomy is not "the best sensor wins" — it is each sensor doing the job it is best at, and a fusion layer combining them so the weaknesses of one are covered by another. Here is the division of labour that suits your exact kit.

| Question the car asks | Primary sensor | Backup / cross-check | Notes |
|---|---|---|---|
| Is there a wall/obstacle in the plane, and how far? | **LiDAR** | Camera depth | LiDAR is metric & reliable in 360°. Trust it for geometry. |
| Is there a *low* or *high* obstacle (curb, box, overhang, drop-off)? | **Camera depth** | — | A single-plane LiDAR is blind to anything off its height. Camera fills this. |
| What is that object — person, car, cone, sign? | **Camera (YOLO)** | — | Semantics are camera-only. Drives "slow near people" rules. |
| Where is the drivable path / lane? | **Camera (seg)** | LiDAR walls | Painted lanes & grass-vs-path are visual. LiDAR bounds the corridor. |
| Where am I on a map? (indoors) | **LiDAR SLAM** | Wheel odom + VO | slam_toolbox scan-matching; GPS is useless indoors. |
| Where am I in the world? (outdoors) | **GPS + compass** | Wheel odom + VO | M8N ≈ 2.5 m accuracy; compass must be calibrated or heading drifts. |
| How far/fast have I moved this instant? | **Wheel odom (VESC)** | Visual odom, IMU | Tachometer `meters_per_tach=0.003424`. Slips on loss of traction. |

> **The mental model.** **LiDAR = skeleton** (accurate shape, no meaning). **Camera = eyes** (meaning + off-plane 3D, but fuzzy metrics). **GPS/compass = a rough world anchor.** **Wheel odom = the inner ear** (short-term motion). Fuse all four; never let any one of them drive alone.

---

## 4 · Recommended architecture

**Layered perception, defense in depth.**

Structure the whole system as four layers running at different speeds. Faster layers are simpler and safer; slower layers are smarter. A failure or slowdown in a smart layer never removes the fast safety reflex underneath it.

| Layer | Runs at | Does | Feeds |
|---|---|---|---|
| **Layer 0 · reflex** | ~50 ms, always on | Depth / LiDAR emergency stop | Overrides everything → brake/freeze |
| **Layer 1 · geometry** | 10 Hz | LiDAR scan → obstacle costmap | Where solid stuff is |
| **Layer 2 · semantics** | ~20 Hz | Front camera → depth + detection + drivable | Enriches the costmap |
| **Layer 3 · planning** | as needed | nav2 global + local planner (map → route → smooth path) | Throttle + steer (VESC + Nano) |

The reactive `autonav.py` you already built is effectively Layer 0+2 fused into one loop. Keep its spirit as the always-on **reflex**, but promote real point-to-point driving to nav2 (Layer 3), and let the camera *feed* the planner rather than steer directly. That is the shift from "avoids things reactively" to "drives to a destination on purpose".

---

## 5 · Which stack — and why nav2

**Reactive is a reflex; nav2 is the plan.**

You asked me to recommend, so here it is plainly. Your goal — **"from one place to another"** — is a *goal-directed* problem: the car needs to know where it is, where the destination is, and choose a route. A reactive camera→steer loop has no concept of a destination; it can only push away from whatever is in front of it. It will happily avoid a wall straight into a dead end.

**Use ROS 2 + nav2:**

- Global planner routes to the goal on a map (SLAM indoors, GPS waypoints outdoors).
- Local planner (Regulated Pure Pursuit or MPPI) produces **smooth**, feasible, Ackermann-aware motion.
- Costmap layers let camera *and* LiDAR both contribute obstacles cleanly.
- Built-in recovery behaviors (back up, clear costmap, spin-analog).
- You already stood up phases 1–4 — this is the path of least new work.

**Keep autonav.py as a reflex:**

- Runs the depth model at low latency as an **independent safety override**.
- If depth says "blocked < X m", it can force a stop even if nav2 hasn't reacted yet.
- Great for bring-up and for the first powered floor test before trusting nav2 at speed.
- Simple, few dependencies, easy to reason about when something goes wrong.

> **The rule you already know, restated.** Only one process may own the VESC at a time. So the reflex and nav2 cannot both send throttle directly. The clean pattern: nav2 owns the actuators; the reflex owns a **veto** — it can command "zero / brake" through the same mux, but never competes for normal driving. One writer, one emergency valve.

---

## 6 · Front camera jobs

**The forward-facing workhorse.**

One image stream, several models. Run them on the same frame (or stagger them across frames — see §13). Each produces a different product that flows into the costmap or the speed governor.

**1 · Metric depth → free-space** *(you have it)* — Depth-Anything V2 (you already run this). Turn the depth map into per-column free distance and a "blocked" flag, and project the ground-plane depth into the costmap so **curbs, boxes, drop-offs and overhangs the LiDAR misses** become obstacles. This is the camera's #1 contribution.

**2 · Object / person / sign detection** *(you have it)* — YOLO11 (TensorRT). People, vehicles, animals, cones, and — on roads — stop/yield signs and traffic lights. Detections become a **dynamic obstacle layer** and trigger rules like "person within 3 m → hard slow". Add a tracker (ByteTrack) so objects persist frame-to-frame.

**3 · Drivable area / lane** *(add)* — TwinLiteNet or YOLOP. On marked roads → lane-keep. On unmarked paths → "grass vs pavement" drivable mask keeps you on the trail. Feeds a **cost gradient** that pulls the path toward the center of the drivable region. This is your mixed indoor/outdoor generality.

**4 · Forward visual odometry** *(optional)* — Camera motion estimation to backstop wheel odometry when tyres slip or indoors where GPS is dead. Monocular VO has a scale ambiguity — resolve it with your wheel-odom scale. Isaac ROS cuVSLAM is GPU-accelerated on Orin, but prefers stereo; treat mono VO as a helper, not a primary.

> **Mounting the front camera.** Aim it slightly downward so the near ground (2–6 m) is well framed — that is where obstacles that matter live for a small car. Record its exact height and pitch relative to `base_link` (this is the URDF offset your hardware doc lists as a TODO); depth-to-ground projection and lane geometry are wrong without it.

---

## 7 · Rear camera jobs

**Not a spare — a dedicated safety & recovery sensor.**

A single rear webcam won't do much while driving forward, but it earns its place the moment the car needs to reverse, recover, or watch its back. Your build already has a dead-end recovery routine — the rear camera is what makes reversing not blind.

- **Reverse perception.** When throttle goes negative (backing out of a dead end, 3-point turn, re-approaching a goal), run depth + a light detector on the rear frame so the car doesn't reverse into a wall, a step, or a person. Your 2D LiDAR sees behind too, but not the low step or the toddler below the scan plane.
- **Recovery behavior eyes.** nav2's "back up" recovery is much safer when a rear free-space check gates it. Wire the rear depth "blocked-behind" flag into the recovery so it refuses to reverse into an obstacle.
- **Rear-approach / tailgation awareness.** Detect something closing from behind (a person, a faster vehicle) and react — useful on shared paths and roads. This is the small-scale analog of the rearview blind-spot / lane-change assist systems in the literature.
- **Backward visual odometry & loop closure.** A rear view sees where you have *been*, which is exactly the signal that helps close loops and stabilize SLAM/VO drift on return trips. Secondary benefit, but free once VO exists.

> **Bandwidth discipline.** Don't run both cameras at 20 fps full-res continuously — you'll fight USB 2.0. Drive the **active-direction** camera at full rate and the other at a trickle (2–5 fps) or gated on gear: forward gear → front full / rear idle; reverse → rear full / front idle.

---

## 8 · Depth & free-space — the deep dive

**Turning one image into "where can I drive".**

Monocular metric depth is the highest-value thing your camera does, because it recovers the 3D structure your single-plane LiDAR simply cannot. The 2024–2025 wave of foundation depth models made this genuinely good and edge-deployable.

### The model landscape (2025)

| Model | What it's good at | On your Orin |
|---|---|---|
| Depth-Anything V2 (small, metric) | Your current choice. Sharp, robust relative + metric depth; huge training set. | **~25 fps @364²**, ~42 fps @308² (ViT-S, TensorRT). Practical sweet spot. |
| Depth-Anything V3 (2025) | Newer generation; a ROS 2 + TensorRT Jetson wrapper now exists. | Watch/trial — potential drop-in upgrade. |
| Metric3D v2 / UniDepth v2 | Strong *metric* accuracy with camera intrinsics; better absolute distances. | Heavier — try only if DA V2 metrics aren't good enough. |
| RTS-Mono (2025) | Real-time *self-supervised* depth aimed at deployment; can fine-tune on your own footage, no depth labels. | Good if you want to specialize to your environment cheaply. |

> **Recommendation.** Stay on **Depth-Anything V2 small, TensorRT, ~308–364²**. It already fits your budget with headroom. Only chase Metric3D/UniDepth if you find the absolute distances too soft after fusing with LiDAR — and evaluate DA V3 as a straight upgrade when convenient.

### The move that actually matters: depth → costmap

A depth map alone doesn't drive the car. Convert it: (1) using the camera's intrinsics + known mount height/pitch, project each pixel's depth onto the ground plane; (2) points that stick up above the ground (or dip below it — a drop-off) become obstacles; (3) drop those obstacle points into a nav2 costmap layer, in the same frame as the LiDAR. Now the planner treats a camera-seen curb identically to a LiDAR-seen wall.

> **The monocular caveat, stated plainly.** Learned mono depth has a **scale ambiguity and drift** — absolute metres wobble, especially far away and in scenes unlike its training data. So: trust LiDAR for absolute distance where the planes overlap, and use camera depth primarily for **off-plane detection and relative free-space**. Where they disagree in the overlap zone, believe the LiDAR.

---

## 9 · Object detection — the deep dive

**Naming the world, so the car can reason about it.**

Detection is what lets rules like "people get a 3 m bubble", "stop at a stop sign", or "a cone is a soft obstacle" exist at all. The good news from the benchmark table: on your Orin this is nearly free.

| Model / format | Latency @640 | Note |
|---|---|---|
| YOLO11n · PyTorch | 15.6 ms | Baseline; don't ship this — export it. |
| YOLO11n · TensorRT FP16 | **4.57 ms** | ~3× faster than PyTorch. Excellent default. |
| YOLO11n · TensorRT INT8 | **3.80 ms** | Fastest; needs calibration data. Verify accuracy holds. |
| YOLO11s · TensorRT | ~7–9 ms (est.) | Noticeably more accurate; you have the headroom — recommended. |

Because a detection costs single-digit milliseconds and your camera only delivers a frame every ~50 ms, **step up to YOLO11s** for better accuracy and still finish long before the next frame arrives. Newer options worth watching: **YOLOv12**, **YOLO26**, and transformer detector **RF-DETR** are the current accuracy leaders (2026) — but YOLO11 has the most mature, battle-tested Jetson/TensorRT path today, so start there.

### Detection needs a tracker and a distance

- **Track, don't just detect.** Add **ByteTrack** so each object keeps an ID across frames — required for "is it getting closer?", "did it stop?", and stable behavior (no flicker between brake/go).
- **Give every detection a distance.** A 2D box has no range on its own. Fuse it: look up the depth (from §8) inside the box, *or* project the LiDAR points into the box. A detection without a distance can't drive a speed decision.
- **Asymmetric risk.** A false "no person" is far worse than a false "person". Bias the person/VRU class toward caution: low threshold, big margin, hard slow-down.

---

## 10 · Lane & drivable area — the deep dive

**Staying on the path, marked or not.**

For "mixed" driving this is the piece that generalizes. Two flavours matter: **lane detection** (the painted lines, on real roads) and **drivable-area segmentation** (the whole surface you *can* drive on — works with no markings, e.g. a dirt path vs grass, a hallway floor vs walls). Drivable area is the more useful of the two for you.

| Model | Size / speed | Accuracy (BDD100K) | Fit |
|---|---|---|---|
| TwinLiteNet | 0.4 M params · 60 fps (Xavier NX) | 91.3% mIoU drivable · 31% IoU lane | Tiny, fast, does both tasks. Strong default. |
| TwinLiteNet+ / variants | config-scalable | higher, tunable | If you want to trade a little speed for accuracy. |
| YOLOP / YOLOPv2 | heavier | detection + drivable + lane in one | One network for 3 tasks; more compute, simpler pipeline. |

Recommendation: **TwinLiteNet for drivable-area** — 0.4 M parameters is almost nothing, it'll run comfortably alongside depth and detection, and 91% drivable-area mIoU is plenty to keep a small car centered on a path. Turn the drivable mask into a soft cost: cheap in the center of the path, expensive toward the edges, so the planner naturally hugs the middle — that is a big part of "smooth".

> **Indoor twist.** Indoors there are no lanes. "Drivable" becomes "floor". The same segmentation model, fine-tuned (or a simple floor/plane classifier), keeps you off walls and out of doorways you shouldn't enter — complementing LiDAR, which sees the walls but not, say, a glossy floor transition or a downward step.

---

## 11 · Localization & visual odometry

**Knowing where you are — the prerequisite for A→B.**

Point-to-point is impossible without decent localization. This is a fusion problem, and the camera is a *contributor*, not the star. Here is the honest hierarchy for your kit:

```
Wheel odometry (VESC tach)      fast, smooth, drifts over distance
        +
IST8310 compass + (opt.) IMU    heading — MUST be calibrated
        +
GPS (M8N) outdoors /            global anchor, corrects drift
LiDAR scan-match indoors
        +
Visual odometry (camera)        optional — helps on slip & indoors
        ↓ fuse
robot_localization EKF  →  one consistent pose
(navsat_transform bridges GPS ↔ map frame)
```

- **Fix the compass first.** Your hardware doc already flags it: uncalibrated + near the motor = corrupt heading. A car that doesn't know which way it's pointing cannot follow GPS waypoints — it'll spiral. 360° hard/soft-iron calibration + mast-mounting is a *prerequisite*, not a nicety, for outdoor A→B.
- **Fuse, don't switch.** Use `robot_localization` (EKF/UKF) to blend wheel odom + heading + GPS/scan-match into one pose. `navsat_transform` converts GPS lat/lon into the map frame for outdoor waypoint goals.
- **Visual odometry — where it helps.** Indoors (GPS dead) and on slippery ground (wheel odom lies), camera VO adds an independent motion estimate. **Isaac ROS Visual SLAM (cuVSLAM)** is GPU-accelerated on Orin and the natural choice — but it's built for stereo; with your single forward camera you get monocular VO, which is scale-ambiguous. Feed it your wheel-odom scale, and treat it as a drift-reducer, not a source of truth.

> **Reality check on GPS.** A bare M8N gives ~2.5 m accuracy — that's *car-lengths* of error for a small RC car. It's fine for "drive to that region of the park", not for "stop precisely at this doorway". For precise outdoor goals you'd need RTK GPS; otherwise, hand off the last few metres to LiDAR/vision local navigation once GPS gets you close.

---

## 12 · Fusing it all together

**How vision and LiDAR meet in one place.**

All the perception outputs must converge somewhere the planner can read. That place is the **nav2 costmap** for obstacles and the **EKF pose** for localization. Keep the fusion boring and explicit — layers, not magic.

| Costmap layer | Fed by | Purpose |
|---|---|---|
| Static layer | SLAM map (indoors) / prior map | Known walls & structure. |
| LiDAR obstacle/voxel layer | RPLIDAR `/scan` | Live geometry, 360°, metric. The backbone. |
| Camera obstacle layer | Depth→ground projection (§8) | Low/high/negative obstacles off the LiDAR plane. |
| Dynamic-object layer | YOLO + tracker + distance (§9) | People/vehicles as inflated, moving keep-out zones. |
| Drivable-area cost | TwinLiteNet mask (§10) | Soft pull toward path center; penalize off-path. |
| Inflation layer | all obstacles | Safety margin sized to your speed & turn radius. |

There is a ready ROS 2 pattern for the detection→LiDAR step (projecting camera boxes onto the laser scan to give them range), and nav2's costmap plugin system is explicitly designed to accept multiple obstacle sources like this. You don't need exotic deep fusion — projective fusion into the costmap is robust and debuggable.

> **Why not "BEV" like a real self-driving car?** Full bird's-eye-view perception (à la Tesla/NuScenes) needs ~6 overlapping cameras to stitch a 360° top-down field. With one forward + one rear camera you can't build a true surround BEV — and you don't need to. Your *LiDAR already provides the 360° top-down geometry*; the cameras' job is to enrich it, not replace it. Projecting front depth into that top-down costmap is your "poor-man's BEV", and it's the right amount of engineering for this platform.

---

## 13 · The compute budget

**Can it all run at once? Yes — with scheduling.**

The frame arrives every ~50 ms (20 fps). Everything below has to finish inside that window, per camera. Here's a realistic front-camera per-frame budget on the Orin:

| Task | Cost | Cadence |
|---|---|---|
| Depth (DA V2 small @308², TensorRT) | ~24 ms | every frame (or 10 Hz) |
| Detection (YOLO11s, TensorRT) | ~7 ms | every frame |
| Drivable area (TwinLiteNet) | ~10–16 ms | every 2nd frame |
| Tracking + costmap projection (CPU) | ~3–5 ms | every frame |
| **Worst-case frame ≈ 24 + 7 + 5 ≈ 36 ms** | | **fits in 50 ms ✓** |

- **Stagger the heavy models.** You don't need depth *and* drivable on the same frame — alternate them. Detection every frame (it's cheap and safety-critical), depth and lane on alternating frames. This keeps every frame under budget with margin.
- **Reduce depth resolution before dropping FPS.** 308² depth (~24 ms) vs 518² (~98 ms) is a 4× speedup for a modest quality loss — usually the right trade on a small car.
- **Keep the reflex on its own thread.** The Layer-0 safety check must never be blocked by a slow model; give it the freshest frame and a hard deadline.
- **Watch memory + thermals, not just FPS.** 8 GB is shared CPU/GPU; multiple TensorRT engines + ROS 2 add up. Your OLED stats screen and `tegrastats` are your friends here.

---

## 14 · Calibration, timing & latency

**The unglamorous work that makes vision usable.**

Great models on badly-calibrated, badly-timed cameras produce confident nonsense. These are the prerequisites that turn pixels into trustworthy geometry — and several tie directly to TODOs already in your hardware doc.

- **Intrinsic calibration.** Checkerboard-calibrate each camera (focal length, principal point, distortion). Required for depth-to-ground projection, VO, and any lane geometry. Uncalibrated → distances and angles are systematically wrong.
- **Extrinsic calibration (the URDF offsets).** Measure each camera's exact position and orientation relative to `base_link` — this is the "measure URDF sensor offsets, wheelbase & max steer angle" item on your list. Without it you can't put camera obstacles in the same frame as LiDAR obstacles, and fusion falls apart.
- **Time synchronization.** Camera, LiDAR and odometry must share a clock. If a camera detection is stamped 100 ms late, you'll place a moving obstacle where it *was*, not where it *is*. Use ROS 2 timestamps + `message_filters` to align streams.
- **Latency = blind distance.** Total pipeline latency (capture → USB → model → costmap → planner → VESC) at speed equals how far you travel "blind". At 3 m/s, 150 ms of latency = 0.45 m committed before the car can react. Measure end-to-end latency and size your *max safe speed* to it. This is the hard link between perception and "how fast is safe".
- **Exposure for motion.** You already pin manual exposure; when moving, drop it further (~156 as your doc notes) to cut motion blur, trading brightness for sharpness. Blur destroys both depth and detection precisely when you're fast.

> **A concrete first-day calibration checklist.** (1) Intrinsics for both cameras. (2) Mount heights + pitch measured and written into the URDF. (3) Compass 360° calibration, mast-mounted. (4) End-to-end latency measured with a timestamp probe. (5) LiDAR↔camera extrinsic verified by overlaying a LiDAR scan on a camera detection. Do these and everything downstream gets dramatically more reliable.

---

## 15 · What a proper self-driving car must see

**The full perception checklist — and who covers each.**

You asked for "everything a proper self-driving car should know and see." Here is the complete ontology, mapped to which of your sensors covers it, and where the honest gaps are.

| Must perceive | Best sensor here | Status on robocar |
|---|---|---|
| Solid obstacles (walls, furniture) in-plane | LiDAR | ✅ covered |
| Low obstacles / curbs / steps up | Camera depth | ⚠️ add depth→costmap |
| Negative obstacles (potholes, drop-offs, downstairs) | Camera depth | ⚠️ add — LiDAR can't |
| Overhangs / low ceilings / bars | Camera depth | ⚠️ add — off LiDAR plane |
| People & vulnerable road users | Camera (YOLO) | ✅ have — add distance+track |
| Vehicles & their motion | Camera + LiDAR | ⚠️ add tracking/prediction |
| Animals, cones, debris | Camera (YOLO) | ✅ have |
| Drivable surface / lane | Camera (seg) | ⚠️ add TwinLiteNet |
| Traffic signs (stop/yield/speed) | Camera (detect/classify) | ⚠️ add — road use |
| Traffic lights | Camera (detect + color) | ⚠️ add — road use |
| Ego position (indoor) | LiDAR SLAM | ✅ phases 1–4 |
| Ego position (outdoor) | GPS + compass | ⚠️ calibrate compass |
| Ego motion / speed | Wheel odom (VESC) | ✅ calibrated |
| Behind the car | Rear camera + LiDAR | ⚠️ wire rear cam |
| Weather/lighting degradation awareness | Camera self-check | ⚠️ add confidence gating |

> **The gaps that bite.** Negative obstacles (a drop-off or downward stair) and overhangs are invisible to a single-plane LiDAR — a robot that only trusts LiDAR will happily drive off a ledge. These are precisely where camera depth is not optional but **safety-critical**. Prioritize depth→costmap for exactly this reason.

---

## 16 · Safety architecture

**"Safer" is a system property, not a model.**

No single model makes the car safe; a layered failure-tolerant design does. Build safety as overlapping nets so any one failing doesn't end in a crash.

- **Independent reflex layer.** A minimal, always-on depth/LiDAR check that can force a stop, running separately from the smart stack (§4). If perception hangs, the reflex still fires.
- **Speed governed by perception quality.** Slow down when: depth range is short, detection confidence is low, the image is dark/blurred, GPS/compass uncertainty is high, or latency spikes. Confidence-aware speed is the core of "fast when safe, cautious when not".
- **Asymmetric caution around people.** Person detected → hard slow / stop with a generous bubble. Over-brake for humans; it's the cheapest insurance you have.
- **Watchdogs everywhere.** You already have the VESC 1 s no-command stop — extend that thinking: a stale camera frame, a dead LiDAR, or a lost localization pose should each drop the car to a safe stop, not coast on last commands.
- **Graceful sensor degradation.** Camera blinded by sun? Fall back to LiDAR-only, reduced speed. LiDAR fails? Camera reflex + crawl. Never a hard "all or nothing".
- **Geofence + e-stop.** Keep the web-console E-STOP reachable, and cap the operating area during testing so a runaway is bounded.

---

## 17 · Faster & smoother

**Where speed and smoothness actually come from.**

Perception *enables* speed (you can only safely go as fast as you can see and react), but smoothness and pace are mostly the **controller's** job. Two levers:

### Smoother

- Use nav2's **Regulated Pure Pursuit** (predictable, gentle) or **MPPI** (optimizes smoothness + obstacle cost directly) as the local controller.
- Add a **velocity smoother** so throttle/steer change gradually — no jerks through the VESC or the servo linkage.
- Filter perception temporally (you already smooth steer) so a one-frame depth glitch doesn't yank the wheel.
- Respect the **Ackermann limits** (your 60/90/115° asymmetric range, wheelbase, min turn radius) in the planner so it never asks for a turn the car can't make.

### Faster (safely)

- **See farther** → a better camera (CSI/global-shutter) and longer-range depth raise the speed ceiling more than any tuning.
- **Curvature-aware speed**: fast on straights, automatically slow into turns — smoother *and* quicker overall than one flat speed.
- **Cut latency** (§14): every 100 ms shaved off the pipeline is speed you can add back at the same safety margin.
- **Confidence-scaled throttle**: open it up on clear, well-lit, well-localized stretches; back off automatically otherwise.

---

## 18 · Model shortlist

**What to actually run — the opinionated picks.**

| Job | Pick | Format | Why |
|---|---|---|---|
| Depth / free-space | Depth-Anything V2 (small, metric) | TensorRT @308–364² | Already yours; fits budget; robust. Watch DA V3. |
| Object detection | YOLO11s (step up from 11n) | TensorRT FP16/INT8 | ~7 ms — headroom buys accuracy. Mature Jetson path. |
| Tracking | ByteTrack | CPU | Stable IDs → closing-distance logic. |
| Drivable / lane | TwinLiteNet | TensorRT | 0.4 M params, 91% drivable mIoU, tiny cost. |
| Visual odometry (opt.) | Isaac ROS cuVSLAM | GPU-accelerated | Best Orin VO/SLAM; mono is scale-limited — fuse wheel scale. |
| Localization fusion | robot_localization + navsat_transform | CPU | Standard, reliable EKF for odom+IMU+GPS. |
| Planner / controller | nav2 + Regulated Pure Pursuit / MPPI | CPU | Real A→B; smooth, Ackermann-aware. |

---

## 19 · Roadmap

**The order I'd build it in.**

Each step is verifiable on its own before the next. This sequences vision work against your existing build so nothing is wasted, and the safety-critical pieces come first.

1. **Do first · prerequisites — Calibrate everything.** Camera intrinsics ×2, camera extrinsics into the URDF, compass 360° calibration + mast mount, end-to-end latency measurement. Nothing downstream is trustworthy without these.
2. **Safety foundation — Depth → costmap projection.** Project front-camera depth onto the ground and feed a nav2 obstacle layer. Immediately closes the negative-obstacle / low-obstacle / overhang gaps LiDAR can't see. Highest safety payoff.
3. **Semantics — YOLO11s + tracker + distance.** Upgrade detection, add ByteTrack, attach a distance (from depth or LiDAR) to every box, and wire the dynamic-object costmap layer + person-slowdown rule.
4. **Path generality — Drivable-area segmentation.** Add TwinLiteNet; turn the mask into a soft center-of-path cost. This is what makes mixed marked/unmarked driving work.
5. **Localization — EKF fusion + GPS waypoints.** robot_localization blending wheel odom + calibrated compass + GPS (outdoor) / scan-match (indoor). Now the car can hold a consistent pose — the real prerequisite for A→B.
6. **Point-to-point — nav2 goal driving + reflex veto.** Global plan to a goal, RPP/MPPI local control, recovery behaviors gated by the rear camera, and the always-on depth reflex holding a veto. First supervised powered floor test at low speed.
7. **Refine — Speed governor + rear camera + VO.** Confidence-scaled speed, curvature-aware pace, rear-camera reverse/recovery, and optional visual odometry for slip/indoor robustness. Now push "faster & smoother".

---

## 20 · Sources

- Depth-Anything on Jetson Orin (FPS/latency by resolution, ViT-S, TensorRT) — [IRCVLab/Depth-Anything-for-Jetson-Orin](https://github.com/IRCVLab/Depth-Anything-for-Jetson-Orin); Depth-Anything V3 ROS 2 + TensorRT Jetson wrapper — [Open Robotics Discourse](https://discourse.openrobotics.org/t/release-gerdsenais-depth-anything-3-ros2-wrapper-with-real-time-tensorrt-for-jetson/52364); TensorRT C++ impl — [spacewalk01/depth-anything-tensorrt](https://github.com/spacewalk01/depth-anything-tensorrt).
- YOLO11 latency on Orin Nano Super (n: PyTorch 15.6 ms, FP16 4.57 ms, INT8 3.80 ms @640) — [Ultralytics NVIDIA Jetson guide](https://docs.ultralytics.com/guides/nvidia-jetson/); [benchmarks blog](https://www.ultralytics.com/blog/ultralytics-yolo11-on-nvidia-jetson-orin-nano-super-fast-and-efficient); detector landscape (YOLOv12/YOLO26/RF-DETR) — [Roboflow](https://blog.roboflow.com/best-object-detection-models/).
- TwinLiteNet (0.4 M params, 91.3% drivable mIoU, 31% lane IoU, 60 fps Xavier NX) — [arXiv:2307.10705](https://arxiv.org/abs/2307.10705).
- Monocular depth for driving / edge (surveys + 2025 real-time methods) — [ScienceDirect systematic review](https://www.sciencedirect.com/science/article/pii/S259012302501429X); [RTS-Mono (arXiv:2511.14107)](https://arxiv.org/html/2511.14107); UAV Jetson depth deployment — [ResearchGate](https://www.researchgate.net/publication/397047206_Edge-Aware_Monocular_Depth_Estimation_for_UAVs_Deployment_of_MiDaS-Small_and_DepthAnythingV2_on_NVIDIA_Jetson_Nano).
- Visual SLAM/odometry on Orin — [Isaac ROS Visual SLAM (cuVSLAM)](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html); VO on Jetson — [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC11858963/).
- Camera–LiDAR fusion & nav2 costmaps — [scan_detection_fusion (ROS 2)](https://github.com/HexboxRC/scan_detection_fusion/tree/main/); [Nav2 sensor setup](https://docs.nav2.org/setup_guides/sensors/setup_sensors.html).
- Rear/blind-spot vision — [MDPI: Rearview camera blind-spot & lane-change assist](https://www.mdpi.com/2076-3417/15/1/419).
- Small-scale autonomous car perception survey — [arXiv:2404.06229](https://arxiv.org/html/2404.06229v2); F1TENTH/RoboRacer platform survey — [ResearchGate](https://www.researchgate.net/publication/392918449_Advancing_Autonomous_Racing_A_Comprehensive_Survey_of_the_RoboRacer_F1TENTH_Platform).

---

*robocar · camera & perception reference · synthesized from 2025–2026 research + your hardware_report.html · 2026-08-08*
