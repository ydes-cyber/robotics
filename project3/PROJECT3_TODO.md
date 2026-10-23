# Project 3 implementation plan (inspection only)

Authority: `spec/FK_API.md`, `spec/ROBOT_DESCRIPTION.md`, and `spec/ROSBRIDGE_PROTOCOL.md`. `docs/PROJECT3_FK.md` supplies grading and testing context. This document is a plan; no Project 3 implementation has been made.

## Source inventory and reuse decision

- `~/rob511/project3` has the handout, three specs, four alternative starters, Fetch URDF and license, and `make_submission.sh`. The Python starter has a placeholder `main.py`, an XML example, and empty `forward_kinematics.py`, `robot_state.py`, `rosbridge.py`, `services.py`, `transforms.py`, `urdf_model.py`, and `urdf_parser.py`.
- There is **no Project 1 source directory under `~/rob511`** in this workspace. Its gateway, middleware, node client, and exact reusable file names therefore cannot yet be verified. Do not claim a Project 1 file has been inspected or copy from an unknown location. Obtain its path/archive before Phase 1 implementation.
- Project 2's `python/src/main.py` contains a TCP listener on `127.0.0.1:9095`, newline reading through `socket.makefile`, JSON serialization, a subscriber map, and a socket send lock. Its service handling is Project 2-specific; `advertise_service`, service routing, and `service_response` are placeholders. The file is 957 lines. It is reference material for patterns, **not** a complete Project 1 middleware to copy wholesale. Project 2 stays untouched.
- `starter/python/src/urdf_example.py` is the course's `xml.etree.ElementTree` example. Reuse its direct-child XML approach. It also strips a leading BOM, whose acceptance is unspecified; no grading behavior should depend on it.
- Fetch is 22,206 bytes: 26 direct-child links, 25 joints (10 fixed, 6 revolute, 5 continuous, 4 prismatic), root `base_link`. It uses mesh visuals, links without visuals, scientific notation, and fixed joints with zero axes (`estop_joint`, `laser_joint`). Mesh assets are absent and unnecessary for FK. Keep its CC BY-NC-SA attribution if redistributed.

## Required contract by delivery stage

### Checkpoint: graded

1. `make build/run/clean`; foreground runtime ready within 10 seconds, including successful small-URDF load and `/tf` and `/xform_world` output. `make run` starts the gateway, parameter server, robot state publisher, and world state publisher. It starts without a robot and does not publish `/joint_states`, `/joint_trajectory`, or `/global_pose`, or set `robot_description`.
2. Generic parameter storage through `/param_server/set_param` and `/param_server/get_param`, with per-name versions and the exact response envelope. `robot_state_publisher` polls `robot_description` through the service, validates every new observed version, publishes description status within 2 seconds, retains the previous robot after rejection, and replaces it atomically after acceptance.
3. All specified URDF syntax, defaults, and validation, including axis and limit validation even at zero configuration. Zero-position local `/tf` transforms for every joint, including fixed, and world matrices for every link, including the root; chains, branches, and real unmodified URDFs. Every hop must support at least 1 MiB lines, although the near-1 MiB description test is final-only.

### Final autograder: graded in addition to checkpoint

1. Near-1 MiB description request/reply; movable-joint FK from `/joint_states`, exact latest-message replacement semantics, unknown/fixed names ignored, untouched position values, and verbatim stamps.
2. Latest `/global_pose` applied to root and all descendants. `/tf` and `/xform_world` complete periodic messages at least 5 Hz, late subscriber delivery within one period, and clean robot reload with no old links. Accuracy: translation `1e-4` m, rotation `1e-4` rad; quaternion norm within `1e-4`; world rotation orthonormal within `1e-4` with determinant `+1`.
3. The same parameter, validity, and zero-configuration features are graded again. Grading weights: parameter/description 15%, zero FK 25%, joint FK 25%, topic/global pose 10%.

### Recommended/demo: not autograded

- `make demo` adds `joint_state_publisher` and `finite_state_machine`. `joint_state_publisher` reads `robot_description`, consumes the last `/joint_trajectory` point, publishes all movable joints on `/joint_states` at about 10 Hz, and preferably provides `/joint_state_publisher/reset`. A proportional servo (gain 50/s, clamped to limits) or snapping to setpoint is permitted; the reference retains targets by name across reloads.
- `finite_state_machine` validates a table of states and named transitions, publishes a JointTrajectory setpoint on state entry, detects arrival by `epsilon`, follows `on_arrival`, and publishes `/fsm/status` at about 10 Hz. Recommended services: `/fsm/update_logic`, `/fsm/get_logic`, `/fsm/set_running`, `/fsm/fire_transition`, `/fsm/save_to_file`, `/fsm/load_from_file`, `/fsm/list_saved`. Saved identifiers should match `[A-Za-z0-9_-]{1,64}` and files live in `fsm/<identifier>.json`. These behaviors are SHOULD; custom demo design is allowed.

### Portfolio: staff graded

- Record a video of a robot performing an FSM choreography (15%); add a portfolio page describing the robot, choreography, and external `/xform_world` viewer (10%). No viewer is supplied. If Fetch is shown, credit “Fetch robot description © Fetch Robotics, Inc., CC BY-NC-SA 4.0”. Portfolio due date is TBD in the handout.

## Wire, node, and data inventory

| Node or component | Required input and state | Required output |
|---|---|---|
| Project 1-style gateway/hub and node client | Concurrent TCP clients on `127.0.0.1:9095`; UTF-8, one JSON object per newline; `advertise`, `unadvertise`, `publish`, `subscribe`, `unsubscribe`, `advertise_service`, `unadvertise_service`, `call_service`, `service_response`; per-connection subscriptions/providers; outstanding call IDs | Current-subscriber delivery; registration `status`; correlated service reply with caller's original opaque ID; clean failure on missing provider/disconnect/timeout; connection cleanup. Type strings are informational. Ignore unknown fields. Malformed input must not kill other connections. No history/latching. Up to 4 MiB protocol line limit; Project 3 must handle at least 1 MiB. |
| `param_server` | Map `name -> (JSON value, version)`; `robot_description` initially absent | `/param_server/set_param`: reject missing/non-string name; otherwise store any JSON value including missing `value` as `null`, increment per-name version starting at 1. `/param_server/get_param`: return exact stored value/version or `null`/0 and false. All responses use object `values`, Boolean `result`, string `status`; failures have non-empty status. It does not validate URDF. |
| `robot_state_publisher` | Poll `get_param(robot_description)`; version, loaded `Robot`, latest joint map and stamp; subscribe `/joint_states` (`sensor_msgs/JointState`) | Set `/robot_state_publisher/description_status` after load decision: `{version,accepted,error,loaded_version,root_link}`; publish `/tf` (`tf/tfMessage`) with complete `transforms` of `TransformStamped`. |
| `robot_world_state_publisher` | Subscribe `/tf` and `/global_pose` (`geometry_msgs/Pose`); hold only latest complete TF batch and latest root pose (identity initially) | Publish `/xform_world` (`xform_world/MatrixTransformArray`) with complete `transforms` of `MatrixTransform`. |
| `joint_state_publisher` (SHOULD) | `robot_description`; `/joint_trajectory` (`trajectory_msgs/JointTrajectory`), last point | `/joint_states`; optional `/joint_state_publisher/reset`. |
| `finite_state_machine` (SHOULD) | FSM document; `/joint_states`; `/fsm/*` services | `/joint_trajectory`; `/fsm/status` with `current_state`, `time_in_state`, `last_transition`, `running`. |

`TransformStamped` has `header.stamp`, parent `header.frame_id`, `child_frame_id`, and `transform.translation.{x,y,z}` plus unit `rotation.{x,y,z,w}`. `MatrixTransform` has the same header/child keys and exactly 16 JSON numbers, column-major (`matrix[4*c+r] = M[r][c]`; translation at indices 12–14; bottom row 0,0,0,1). `/global_pose` has `position.{x,y,z}` and unit `orientation.{x,y,z,w}`; ignore extra fields. Every `/tf` batch uses the latest `/joint_states` stamp (zero before first message), and every world batch uses its input TF stamp. Frame names are link names except `global_frame`, which is z-up. No viewer axis correction belongs in the runtime.

Suggested internal data: `Link(name, first_visual?)`, `Joint(name, type, parent, child, origin_xyz, origin_rpy, axis, limit?)`, `Robot(name, links, joints, root_link, children_by_parent)`, `Transform(translation, rotation)`, per-name parameter `{value,version}`, joint state `{positions_by_name, stamp}`, and the latest TF edge tree. Function and class names below are proposed implementation names; the specs mandate behavior and wire names, **not** Python symbol names.

## URDF parsing and validation checklist

- Accept well-formed UTF-8 XML 1.0, leading whitespace, declaration and processing instructions, comments, CDATA, CRLF, both quote styles, self-closing/paired tags, predefined and numeric entities, namespace declarations, and unknown elements/attributes. Flat URDF with a namespace declaration must load; ElementTree expands default namespaces into tag names, so account for this while recognizing direct children. Namespaced elements such as `<xacro:link>` are unspecified. Unexpanded xacro, JRDF, mesh loading, and `package://` resolution are out of scope.
- Read only direct `<robot>` children named `<link>` and `<joint>`. Root tag must be `robot`; its optional name defaults to `robot`. Require at least one link. Link names, joint name/type, and joint direct-child parent/child with `link` attributes are required. Parent and child must be among the links. Exactly one root link (never a joint child) is required; order in XML is irrelevant.
- Joint type is exactly `revolute`, `continuous`, `prismatic`, or `fixed`. Optional origin `xyz`/`rpy` default to `(0,0,0)` even when the element/attribute is absent. Optional axis `xyz` defaults to `(1,0,0)`, including `<axis/>`; a present malformed axis is rejected even on a fixed joint. For revolute/prismatic require `<limit>`; absent `lower` or `upper` defaults to 0 and present values must be numeric. Ignore `<limit>` entirely on continuous/fixed, even if malformed. Limits do not clamp FK. Ignore mimic behavior, inertial, collision, gazebo, transmission, and other unknown content.
- Only the link's first `<visual>` matters. If present, require `<geometry>` and one of: box `size` triple; cylinder numeric `radius` and `length`; sphere numeric `radius`; mesh `filename` and optional numeric triple `scale`. Validate optional visual origin as joint origin; material is ignored. Visuals have no FK effect. Later visuals are unspecified.
- Vector attributes contain exactly three decimal literals separated by XML whitespace, allowing leading/trailing whitespace. A decimal literal matching `[+-]?([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?` must be accepted if finite; underflow is zero. Reject bad count, empty present value, commas, underscores, and nonnumeric text. Hex floats, `nan`/`inf`, and overflow are unspecified.
- Reject the nine `ROBOT_DESCRIPTION.md` Validity categories: non-string/empty/XML/root; no link; missing names/type/parent/child attributes; unsupported type; missing link reference; nonunique root; missing required limit or bad bound; bad vectors/numbers; invalid first visual. Validate the entire candidate before changing the loaded robot. Duplicates, self edges, multiple parents, cycles, zero/nonfinite movable axis, and nonfinite numbers are unspecified but must not crash or hang. A fixed joint's zero axis **must** be accepted.

## Transform and FK checklist

- Use 4x4 homogeneous transforms with composition `T_parent_child = T_origin · T_motion(q)` and `T_global_child = T_global_parent · T_parent_child`. `T_origin = Trans(xyz) · Rot(rpy)`. Fixed-axis RPY is `Rz(yaw) · Ry(pitch) · Rx(roll)`; beware transposition.
- Revolute and continuous motion is axis-angle rotation by `q` radians about the **normalized** joint-frame axis. Prismatic motion is translation `q * axis` with axis **not normalized**. Fixed motion is identity. Do not wrap continuous angles or clamp revolute/prismatic values.
- Compose paths by traversing the tree from the computed root, visiting branches and fixed edges. The root has no TF edge but has a world matrix. TF publishes local parent-to-child transforms, one per joint; world output publishes absolute transforms, one per link. World root pose is identity until `/global_pose`, then `Trans(position) · Rot(orientation)`.
- Produce unit quaternions, including near 180° rotations (use stable four-branch matrix conversion or direct quaternion composition). Quaternion sign is unrestricted. Ensure JSON numbers are finite and serialize names with `json.dumps`.

## Proposed Python files (each `.py` below 300 lines)

The implementation stays inside `starter/python/`, because `make_submission.sh python` archives only that directory. Use existing empty modules before adding files. Target less than 250 lines per file to leave room; check `wc -l` during implementation. Preserve the Project 1 middleware's actual roles once its source is available rather than redesigning it for this list.

```text
starter/python/
  Makefile                         # modify existing targets
  src/
    main.py                        # replace placeholder with foreground lifecycle
    rosbridge.py                   # gateway/hub, adapted from actual Project 1 source
    node_client.py                 # add only if Project 1 client needs its own file
    services.py                    # param_server service handlers and state
    urdf_example.py                # retain course XML example
    urdf_model.py                  # typed Robot/Link/Joint data and tree
    urdf_parser.py                 # XML interpretation and validation
    transforms.py                  # 3D transform and quaternion math
    forward_kinematics.py          # joint/local/world FK and wire conversion
    robot_state.py                 # robot_state_publisher
    world_state.py                 # add for robot_world_state_publisher
    joint_state_publisher.py       # add only for demo phase
    finite_state_machine.py       # add only for demo phase
    fsm_storage.py                 # add only if FSM module approaches 300 lines
```

Copy/reuse **after locating Project 1**: its gateway, pub/sub hub, service routing, TCP JSON framing, node client, and startup glue into the corresponding Python files, with only changes needed for the Project 3 contract and line-size/scope fixes. Exact Project 1 source paths are unresolved. Modify the existing Python `Makefile`, `main.py`, and empty modules above; add `node_client.py` and `world_state.py` only when needed. Do not copy Project 2's project-specific physics, IK, or service code. Do not modify Project 1, Project 2, or alternate-language starters.

## Dependency-ordered implementation tasks

Each row names the proposed file and symbol, behavior, requirement, dependencies, and a concrete test. Symbol names may change to match the recovered Project 1 API.

| Phase / task | File; function/class | Behavior and relevant requirement | Depends on | Test |
|---|---|---|---|---|
| 1.1 Recover middleware | Project 1 source → `src/rosbridge.py`, possible `src/node_client.py`; existing gateway/hub/client symbols | Locate and inspect actual Project 1 gateway, pub/sub, service routing, node client, and Makefile; copy only reusable code into Project 3; retain protocol and process roles. | Project 1 source path from user | Inventory symbols and compare with `spec/ROSBRIDGE_PROTOCOL.md`; no edits to old projects. |
| 1.2 Robust framing | `src/rosbridge.py`, possible `src/node_client.py`; `read_message`, `send_message` or recovered equivalents | Read until newline with growing buffer, UTF-8/JSON; serialize outbound frames; cover external request, service-forward, service-reply, and pub/sub hops at ≥1 MiB (protocol may reject >4 MiB). Isolate bad clients; concurrent clients, call IDs, cleanup, slow clients. Set `SO_REUSEADDR`. | 1.1 | External concurrent client test; send >64 KiB and near-1 MiB JSON; inspect every hop and restart port 9095 immediately. |
| 1.3 Runtime | `src/main.py`; `main`, lifecycle helpers; `Makefile` targets `build`, `run`, `clean` | Foreground, offline, repeatable build under 180 s; start gateway and three graded nodes in <10 s, with no robot and no demo publishers; clean generated files. | 1.1–1.2 | `make build`, `make run`, connect on 9095; load a small URDF within 10 s after full pipeline exists. |
| 2.1 Parameter server | `src/services.py`; `ParameterServer`, `set_param`, `get_param` | Per-name JSON value/version, missing value as null, invalid name rejection, exact service envelopes; no URDF validation. | 1.1–1.3 | External calls for two names, repeated sets, absent get, missing/non-string names, malformed URDF value stored unchanged. |
| 3.1 Robot model | `src/urdf_model.py`; `Link`, `Joint`, `Robot`, `build_tree` | Hold parsed data, child adjacency, unique root from links minus joint children; safe traversal data. | 2.1 only for integration | Unit fixtures for root not first, chain, branch, missing/multiple root; avoid hang on unspecified cycle. |
| 3.2 Numeric and XML parsing | `src/urdf_parser.py`; `parse_number`, `parse_vector3`, `parse_origin`, `parse_visual`, `parse_robot_description` | ElementTree direct children; decimal spelling/count; required attributes, defaults, all four types, limits, first visual shapes, whole-description validity. | 3.1 | One valid/default/XML-style fixture and one rejection per nine validity categories; load Fetch unmodified. |
| 4.1 Transform math | `src/transforms.py`; `identity`, `translation`, `rpy_rotation`, `axis_angle`, `quaternion_to_matrix`, `matrix_to_quaternion`, `compose`, `flatten_column_major` | 4x4 math, fixed-axis RPY, axis-angle, stable unit quaternions, column-major output; no external robotics/transform library. | none | Hand-check each RPY axis and combined RPY, 180° rotation, composition order, orthonormality/determinant, translation slots. |
| 5.1 Zero FK | `src/forward_kinematics.py`; `joint_transform`, `local_transforms`, `world_transforms`, `to_tf_message`, `to_world_message` | At q=0 compute all local joint transforms, tree traversal from root, include fixed and root world; correct wire structures and finite JSON numbers. | 3.1–4.1 | Hand-computed translated/rotated chain and branch; 25 TF and 26 world entries for Fetch. |
| 5.2 Description lifecycle | `src/robot_state.py`; `RobotStatePublisher.poll_description`, `load_description`, `publish_status` | Poll `get_param`; process each new observed version (latest may skip faster arrivals); report within 2 s; validate candidate then swap robot, preserve old on rejection; status written only after state is effective. | 2.1, 3.2, 5.1 | Set valid/invalid/valid URDFs, compare returned versions to status, inspect retained/replaced `/tf`. |
| 6.1 TF publisher | `src/robot_state.py`; `RobotStatePublisher.publish_tf`, `run` | Complete `/tf` at ≥5 Hz while loaded, no TF before load, current stamp, late subscribers served within period. | 5.1–5.2 | External subscribe before/after load; count messages over 2 s; verify one edge per joint. |
| 7.1 World publisher | `src/world_state.py`; `RobotWorldStatePublisher.on_tf`, `on_global_pose`, `publish_world`, `run` | From latest TF batch **alone** determine root and compose every link with latest global root pose; complete column-major `/xform_world` ≥5 Hz, stamp copied from TF. | 4.1, 6.1 | Root identity and each link pose; publish global pose; late subscribe; reload different robot and assert no old links. |
| 8.1 Joint-state FK | `src/forward_kinematics.py`; `joint_transform`, `local_transforms` (extend); `src/robot_state.py`; `on_joint_states` | Revolute/continuous normalized axis, prismatic raw axis, fixed identity; each message resets absent joints to zero, ignores unknown/fixed, preserves exact q and stamp. | 4.1, 6.1–7.1 | Non-unit general revolute axis; rotated-origin prismatic axis `2 0 0`; continuous >π; partial/unknown names; no clamp/wrap; stamp. |
| 9.1 Checkpoint test | `starter/python/tests/test_checkpoint.py` (new only if useful); external TCP client test helpers | Exercise service envelope, every URDF validity/default/XML case, reload atomicity, zero FK chain/branch/Fetch, 10 s readiness. | 1–7 | `make run`; run external socket script; compare transforms to hand values and statuses to returned versions. |
| 10.1 Final test | `starter/python/tests/test_final.py` (new only if useful); external TCP client test helpers | Exercise near-1-MiB request/reply, all joint motion, global pose, rate, late subscriber, reload, numerical tolerances, names with JSON escaping. | 8–9 | `make run`; run external socket script and inspect `/tf`, `/xform_world`, status; check all `.py` files <300 lines. |
| 10.2 Package | `make_submission.sh` (use as-is); `starter/python/Makefile` | Create `submission.tar.gz` with `Makefile` at archive root; verify offline build/run/clean from archive. Do not add a `map` target. | 10.1 | `./make_submission.sh python`, `tar -tzf submission.tar.gz`, extract in temporary directory and run targets. |
| 11.1 Demo joint driver | `src/joint_state_publisher.py`; `JointStatePublisher.on_trajectory`, `poll_description`, `publish_joint_states`, optional `reset` | Last trajectory point by joint name, servo/snap, all movable joints at ~10 Hz, optional reset and reload continuity. | 2–8 | `make demo`; send trajectory and observe joint motion in `/joint_states` and `/xform_world`. |
| 11.2 Demo FSM | `src/finite_state_machine.py`; `FiniteStateMachine.validate_logic`, `enter_state`, `on_joint_states`, `fire_transition`, service handlers; optional `src/fsm_storage.py` | State table, arrival/transition rules, recommended `/fsm/*` services, `/fsm/status`, safe filename identifier. | 11.1 | Load example logic; verify trajectory on entry, epsilon arrival, manual transition, save/load/list and path validation. |
| 11.3 Demo startup | `starter/python/Makefile` target `demo`; `src/main.py` demo mode | Start graded runtime plus recommended nodes, keeping `run` grading behavior intact. | 11.1–11.2 | `make demo`; confirm all recommended services/topics and no conflicts. |
| 11.4 Portfolio | External viewer/page and media, path chosen later | Draw links from `/xform_world`, record FSM choreography, describe robot/viewer, credit Fetch if used. | 11.3 | Watch recorded video and verify portfolio page and attribution. |

## Required implementation/testing guards from course pitfalls

1. Search all copied and new socket code for fixed `recv(n)`/line limits; test both URDF request and `get_param` reply above 64 KiB and near 1 MiB. Check gateway reader/writer, hub forwarding, service provider's reader/writer, `robot_state_publisher` client reader/writer, and subscriber delivery. `socket.makefile().readline()` without a small limit can grow, but all other paths still need review.
2. Test nested fake `<joint>` under `<gazebo>` and `<transmission>`; only direct children count. Test root listed last.
3. Test RPY x, y, z, combined order and nontransposed matrix; parent·joint and origin·motion order with rotated origin.
4. Test default axis `(1,0,0)`, normalized revolute/continuous axis, raw prismatic axis, and fixed zero axis acceptance.
5. Assert fixed joints appear in TF, root and every fixed-child link appear in world output.
6. On invalid reload, compare entire prior TF/world link sets and status; on valid reload, ensure status follows atomic swap and old links disappear.
7. Test partial `/joint_states` resetting unnamed joints, verbatim input stamp, no limit clamp or continuous wrap, and version comparisons relative to `set_param` result.
8. Count complete TF/world messages with no changes and with a late subscriber; require ≥5 Hz and first complete batch within one period.
9. Serialize only through Python JSON library; test names containing quotes, ampersands, and `<`; reject/avoid nonfinite JSON output.
10. Test stable quaternion conversion near 180° and numerical accuracy/orthonormality. Restart runtime quickly to verify `SO_REUSEADDR`; verify pipeline readiness within 10 s.

## Ambiguities and specified latitude

- The Project 1 source location and exact reusable file names are unresolved in this workspace. Phase 1 needs the real source before choosing copy boundaries. Project 2's partial gateway is only a fallback reference.
- The specs do not mandate Python function/class names, process topology, ordering of entries, quaternion sign, status error text, latching, or behavior for a jointless robot's world output. They allow polling and skipping intermediate description versions if updates outpace it, but require processing the latest.
- Duplicates, malformed joint-state arrays/stamps, state retention across a robot reload, zero/nonfinite movable axis, cycles, and some XML/number edge cases are explicitly UNSPECIFIED. Implement defensively without treating a guessed behavior as graded.
