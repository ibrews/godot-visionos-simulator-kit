/**************************************************************************/
/*  simhands_bridge.mm                                                    */
/**************************************************************************/
/*  SIMULATOR-ONLY dev/test bridge: ingest VisionOS-SimHands MediaPipe    */
/*  hand-tracking over MultipeerConnectivity and feed Godot's             */
/*  XRHandTracker, so a custom-Metal immersive Godot app gets articulated */
/*  grab/pinch input in the visionOS Simulator (which delivers no ARKit   */
/*  hand anchors). Completely inert on device and when GODOT_SIMHANDS is  */
/*  unset. On real Apple Vision Pro, ARKit hand tracking already works —  */
/*  this path must never run there.                                       */
/*                                                                        */
/*  Mechanism: SimHands' macOS helper advertises an MCSession over the    */
/*  serviceType "Bonjour" and broadcasts JSON                             */
/*    {"landmarks":[[{x,y,z}x21],...],                                    */
/*     "handednesses":[[{"displayName":"Left"|"Right"}],...]}             */
/*  We are a native MultipeerConnectivity *client* that connects, parses  */
/*  that stream, and fills hand_interaction_states[] each frame. The      */
/*  existing apply_hand_states_to_trackers() then writes the trackers     */
/*  exactly as it does for real ARKit data — so the whole tracker-write   */
/*  path is reused verbatim.                                              */
/**************************************************************************/

#ifdef VISIONOS_ENABLED

#include "visionos_xr_interface.h"

#import <TargetConditionals.h>

#if TARGET_OS_SIMULATOR

#include "core/io/config_file.h"
#include "core/os/os.h"

#import <Foundation/Foundation.h>
#import <MultipeerConnectivity/MultipeerConnectivity.h>
#import <os/lock.h>
#import <os/log.h>

// ---------------------------------------------------------------------------
// Tunables (first-light defaults; exact calibration is human-in-the-loop —
// see REVIEW_NEEDED.md). All metric (meters), all in head-relative space; the
// engine's apply_floor_offset() then lifts y by eye_height (~1.7 roomscale),
// so a wrist y of -0.10 lands ~eye level in front of the XR origin.
// ---------------------------------------------------------------------------
// Compiled-in first-light defaults. The live values below are tuned at RUNTIME from
// user://simhands_calibration.cfg (written by simulator_input.gd on UDP "K*" verbs from the
// SimControlPanel macOS dev tool), so placement/scale can be dialed in while watching the sim
// with no rebuild per tweak. Deleting the cfg (panel "Reset") restores these.
static const float SIMHANDS_DEF_KNUCKLE_M = 0.09f;
static const float SIMHANDS_DEF_PLANE_M = 0.55f;
static const float SIMHANDS_DEF_DEPTH_M = 0.45f;
static const float SIMHANDS_DEF_Y_OFFSET_M = -0.10f;
static const float SIMHANDS_DEF_Z_SHAPE_GAIN = 1.0f;

static float SIMHANDS_HAND_KNUCKLE_M = SIMHANDS_DEF_KNUCKLE_M; // target wrist->middle-knuckle length (self-normalize → metric hand)
static float SIMHANDS_PLANE_M = SIMHANDS_DEF_PLANE_M; // image plane → head-space placement scale (hand x,y in ~±0.275 m)
static float SIMHANDS_DEPTH_M = SIMHANDS_DEF_DEPTH_M; // wrist distance in front of the XR origin
static float SIMHANDS_Y_OFFSET_M = SIMHANDS_DEF_Y_OFFSET_M; // head-relative vertical offset before floor offset
static float SIMHANDS_Z_SHAPE_GAIN = SIMHANDS_DEF_Z_SHAPE_GAIN; // how much MediaPipe z drives finger-curl depth

// Pinch thresholds for the controller "trigger" proxy (local copy of the
// engine's VISIONOS_PINCH_* — the game itself reads the hand-tracker joint
// distance directly, so these only drive the controller-tracker trigger).
static const float SIMHANDS_PINCH_PRESS_M = 0.025f;
static const float SIMHANDS_PINCH_ANALOG_MAX_M = 0.06f;

// MediaPipe landmark index (0..20) → Godot XRHandTracker joint. MediaPipe has
// no separate finger metacarpals; those Godot joints stay untracked (the mesh
// driver skips them). The fingertips + wrist — all the game's pinch logic
// needs — map exactly (wrist=1, thumb_tip=5, index_tip=10, ...).
static const XRHandTracker::HandJoint SIMHANDS_JOINT_MAP[21] = {
	XRHandTracker::HAND_JOINT_WRIST, // 0  wrist
	XRHandTracker::HAND_JOINT_THUMB_METACARPAL, // 1  thumb CMC
	XRHandTracker::HAND_JOINT_THUMB_PHALANX_PROXIMAL, // 2  thumb MCP
	XRHandTracker::HAND_JOINT_THUMB_PHALANX_DISTAL, // 3  thumb IP
	XRHandTracker::HAND_JOINT_THUMB_TIP, // 4  thumb tip
	XRHandTracker::HAND_JOINT_INDEX_FINGER_PHALANX_PROXIMAL, // 5  index MCP (knuckle)
	XRHandTracker::HAND_JOINT_INDEX_FINGER_PHALANX_INTERMEDIATE, // 6  index PIP
	XRHandTracker::HAND_JOINT_INDEX_FINGER_PHALANX_DISTAL, // 7  index DIP
	XRHandTracker::HAND_JOINT_INDEX_FINGER_TIP, // 8  index tip
	XRHandTracker::HAND_JOINT_MIDDLE_FINGER_PHALANX_PROXIMAL, // 9  middle MCP (knuckle)
	XRHandTracker::HAND_JOINT_MIDDLE_FINGER_PHALANX_INTERMEDIATE, // 10 middle PIP
	XRHandTracker::HAND_JOINT_MIDDLE_FINGER_PHALANX_DISTAL, // 11 middle DIP
	XRHandTracker::HAND_JOINT_MIDDLE_FINGER_TIP, // 12 middle tip
	XRHandTracker::HAND_JOINT_RING_FINGER_PHALANX_PROXIMAL, // 13 ring MCP
	XRHandTracker::HAND_JOINT_RING_FINGER_PHALANX_INTERMEDIATE, // 14 ring PIP
	XRHandTracker::HAND_JOINT_RING_FINGER_PHALANX_DISTAL, // 15 ring DIP
	XRHandTracker::HAND_JOINT_RING_FINGER_TIP, // 16 ring tip
	XRHandTracker::HAND_JOINT_PINKY_FINGER_PHALANX_PROXIMAL, // 17 pinky MCP
	XRHandTracker::HAND_JOINT_PINKY_FINGER_PHALANX_INTERMEDIATE, // 18 pinky PIP
	XRHandTracker::HAND_JOINT_PINKY_FINGER_PHALANX_DISTAL, // 19 pinky DIP
	XRHandTracker::HAND_JOINT_PINKY_FINGER_TIP, // 20 pinky tip
};

// Per-joint bone direction = position[DIR_TO] - position[DIR_FROM] (MediaPipe indices).
// For finger/thumb segments this points toward the child joint; for the 5 fingertips it
// continues the parent segment's direction; the wrist points up the hand (→ middle knuckle).
// Used to synthesize joint ORIENTATION (the mesh skins from each joint's basis; without it,
// correctly-placed bones twist the skin → the "distorted claw"). Godot's humanoid-rig
// convention (what hand_mesh_driver's _bone_correction is tuned for) = bone's local +Y points
// distally, so we put +Y along this direction.
static const int SIMHANDS_DIR_FROM[21] = { 0, 1, 2, 3, 3, 5, 6, 7, 7, 9, 10, 11, 11, 13, 14, 15, 15, 17, 18, 19, 19 };
static const int SIMHANDS_DIR_TO[21] = { 9, 2, 3, 4, 4, 6, 7, 8, 8, 10, 11, 12, 12, 14, 15, 16, 16, 18, 19, 20, 20 };

// Plain-old-data carried from the MC delegate queue → the engine main thread.
struct SimHandsLandmark {
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
};
struct SimHandsPodHand {
	bool present = false;
	int side = -1; // 0 = Godot left, 1 = Godot right (after handedness swap)
	SimHandsLandmark lm[21];
};
struct SimHandsPodFrame {
	SimHandsPodHand hands[2];
	uint32_t hand_count = 0;
	uint64_t seq = 0;
};

static os_log_t simhands_log() {
	static os_log_t log = os_log_create("com.agilelens.godotvisionpilot", "simhands");
	return log;
}

// Build a joint orientation: local +Y points along p_distal (toward the child joint), with
// p_ref_up (the palm normal) resolving the roll about the bone. Right-handed; falls back to an
// arbitrary perpendicular if p_ref_up is parallel to the bone. Matches the humanoid-rig
// convention hand_mesh_driver expects (it then applies its per-hand _bone_correction Y-twist).
static Basis simhands_basis_from_dir(const Vector3 &p_distal, const Vector3 &p_ref_up) {
	Vector3 y = p_distal;
	const float yl = y.length();
	if (yl < 1e-5f) {
		return Basis();
	}
	y /= yl;
	Vector3 z = p_ref_up - y * p_ref_up.dot(y); // component of ref ⟂ to the bone
	if (z.length() < 1e-5f) {
		z = y.cross(Vector3(1, 0, 0));
		if (z.length() < 1e-5f) {
			z = y.cross(Vector3(0, 0, 1));
		}
	}
	z = z.normalized();
	Vector3 x = y.cross(z).normalized();
	z = x.cross(y).normalized(); // re-orthogonalize, keep right-handed
	Basis b;
	b.set_column(0, x);
	b.set_column(1, y);
	b.set_column(2, z);
	return b;
}

// ---------------------------------------------------------------------------
// Native MultipeerConnectivity client. Mirrors SimHands' BonjourSession with
// usage = .combined: it both advertises and browses for the "Bonjour" service,
// auto-invites discovered peers, and auto-accepts invitations — so it connects
// to SimHands' helper (or our canned sender) regardless of which side initiates.
// JSON parsing runs on MC's delegate queue; the parsed frame is handed to the
// engine thread under an os_unfair_lock.
// ---------------------------------------------------------------------------
@interface GodotSimHandsClient : NSObject <MCSessionDelegate, MCNearbyServiceBrowserDelegate, MCNearbyServiceAdvertiserDelegate> {
	MCPeerID *_peerID;
	MCSession *_session;
	MCNearbyServiceBrowser *_browser;
	MCNearbyServiceAdvertiser *_advertiser;
	NSString *_serviceType;
	os_unfair_lock _lock;
	SimHandsPodFrame _latest;
	BOOL _hasFrame;
}
- (instancetype)initWithService:(NSString *)serviceType;
- (void)start;
- (void)stop;
- (BOOL)copyLatestFrame:(SimHandsPodFrame *)outFrame;
@end

@implementation GodotSimHandsClient

- (instancetype)initWithService:(NSString *)serviceType {
	self = [super init];
	if (self) {
		_serviceType = serviceType;
		_lock = OS_UNFAIR_LOCK_INIT;
		_hasFrame = NO;
		_peerID = [[MCPeerID alloc] initWithDisplayName:@"GodotVisionPilot"];
		_session = [[MCSession alloc] initWithPeer:_peerID securityIdentity:nil encryptionPreference:MCEncryptionNone];
		_session.delegate = self;
		_browser = [[MCNearbyServiceBrowser alloc] initWithPeer:_peerID serviceType:_serviceType];
		_browser.delegate = self;
		_advertiser = [[MCNearbyServiceAdvertiser alloc] initWithPeer:_peerID discoveryInfo:nil serviceType:_serviceType];
		_advertiser.delegate = self;
	}
	return self;
}

- (void)start {
	[_advertiser startAdvertisingPeer];
	[_browser startBrowsingForPeers];
	os_log(simhands_log(), "[SimHands] client started: advertising + browsing service '%{public}@'", _serviceType);
}

- (void)stop {
	[_advertiser stopAdvertisingPeer];
	[_browser stopBrowsingForPeers];
	[_session disconnect];
	os_log(simhands_log(), "[SimHands] client stopped");
}

- (BOOL)copyLatestFrame:(SimHandsPodFrame *)outFrame {
	BOOL ok = NO;
	os_unfair_lock_lock(&_lock);
	if (_hasFrame) {
		*outFrame = _latest;
		ok = YES;
	}
	os_unfair_lock_unlock(&_lock);
	return ok;
}

// --- MCNearbyServiceBrowserDelegate ---
- (void)browser:(MCNearbyServiceBrowser *)browser foundPeer:(MCPeerID *)peerID withDiscoveryInfo:(NSDictionary<NSString *, NSString *> *)info {
	os_log(simhands_log(), "[SimHands] found peer '%{public}@' — inviting", peerID.displayName);
	[browser invitePeer:peerID toSession:_session withContext:nil timeout:10.0];
}

- (void)browser:(MCNearbyServiceBrowser *)browser lostPeer:(MCPeerID *)peerID {
	os_log(simhands_log(), "[SimHands] lost peer '%{public}@'", peerID.displayName);
}

- (void)browser:(MCNearbyServiceBrowser *)browser didNotStartBrowsingForPeers:(NSError *)error {
	os_log_error(simhands_log(), "[SimHands] browse failed: %{public}@", error.localizedDescription);
}

// --- MCNearbyServiceAdvertiserDelegate ---
- (void)advertiser:(MCNearbyServiceAdvertiser *)advertiser didReceiveInvitationFromPeer:(MCPeerID *)peerID withContext:(NSData *)context invitationHandler:(void (^)(BOOL, MCSession *_Nullable))invitationHandler {
	os_log(simhands_log(), "[SimHands] invitation from '%{public}@' — accepting", peerID.displayName);
	invitationHandler(YES, _session);
}

- (void)advertiser:(MCNearbyServiceAdvertiser *)advertiser didNotStartAdvertisingPeer:(NSError *)error {
	os_log_error(simhands_log(), "[SimHands] advertise failed: %{public}@", error.localizedDescription);
}

// --- MCSessionDelegate ---
- (void)session:(MCSession *)session peer:(MCPeerID *)peerID didChangeState:(MCSessionState)state {
	const char *s = (state == MCSessionStateConnected) ? "connected" : (state == MCSessionStateConnecting) ? "connecting"
																									  : "notConnected";
	os_log(simhands_log(), "[SimHands] peer '%{public}@' → %{public}s", peerID.displayName, s);
}

- (void)session:(MCSession *)session didReceiveData:(NSData *)data fromPeer:(MCPeerID *)peerID {
	NSError *err = nil;
	id obj = [NSJSONSerialization JSONObjectWithData:data options:0 error:&err];
	if (![obj isKindOfClass:[NSDictionary class]]) {
		return;
	}
	NSDictionary *root = (NSDictionary *)obj;
	NSArray *landmarks = root[@"landmarks"];
	NSArray *handed = root[@"handednesses"];
	if (![landmarks isKindOfClass:[NSArray class]]) {
		return;
	}

	SimHandsPodFrame frame;
	NSUInteger nhands = MIN((NSUInteger)2, landmarks.count);
	for (NSUInteger h = 0; h < nhands; h++) {
		NSArray *joints = landmarks[h];
		if (![joints isKindOfClass:[NSArray class]]) {
			continue;
		}
		SimHandsPodHand &hand = frame.hands[h];
		hand.present = true;
		hand.side = -1;
		NSUInteger njoints = MIN((NSUInteger)21, joints.count);
		for (NSUInteger j = 0; j < njoints; j++) {
			id jd = joints[j];
			if (![jd isKindOfClass:[NSDictionary class]]) {
				continue;
			}
			hand.lm[j].x = [((NSDictionary *)jd)[@"x"] floatValue];
			hand.lm[j].y = [((NSDictionary *)jd)[@"y"] floatValue];
			hand.lm[j].z = [((NSDictionary *)jd)[@"z"] floatValue];
		}

		// Handedness: MediaPipe reports L/R swapped; SimHands swaps it back, so
		// displayName "Left" → Godot RIGHT tracker, "Right" → Godot LEFT.
		if ([handed isKindOfClass:[NSArray class]] && h < handed.count) {
			id hi = handed[h];
			if ([hi isKindOfClass:[NSArray class]] && [(NSArray *)hi count] > 0) {
				id info = ((NSArray *)hi)[0];
				if ([info isKindOfClass:[NSDictionary class]]) {
					NSString *name = ((NSDictionary *)info)[@"displayName"];
					if ([name isKindOfClass:[NSString class]]) {
						if ([name isEqualToString:@"Left"]) {
							hand.side = 1;
						} else if ([name isEqualToString:@"Right"]) {
							hand.side = 0;
						}
					}
				}
			}
		}
		if (hand.side < 0) {
			hand.side = (int)h; // fallback: detection order (0→left, 1→right)
		}
	}
	// Disambiguate if both hands resolved to the same side.
	if (nhands == 2 && frame.hands[0].present && frame.hands[1].present && frame.hands[0].side == frame.hands[1].side) {
		frame.hands[1].side = 1 - frame.hands[0].side;
	}
	frame.hand_count = (uint32_t)nhands;

	os_unfair_lock_lock(&_lock);
	frame.seq = _latest.seq + 1;
	_latest = frame;
	_hasFrame = YES;
	os_unfair_lock_unlock(&_lock);
}

// Required MCSessionDelegate methods we don't use.
- (void)session:(MCSession *)session didReceiveStream:(NSInputStream *)stream withName:(NSString *)streamName fromPeer:(MCPeerID *)peerID {}
- (void)session:(MCSession *)session didStartReceivingResourceWithName:(NSString *)resourceName fromPeer:(MCPeerID *)peerID withProgress:(NSProgress *)progress {}
- (void)session:(MCSession *)session didFinishReceivingResourceWithName:(NSString *)resourceName fromPeer:(MCPeerID *)peerID atURL:(NSURL *)localURL withError:(NSError *)error {}
- (void)session:(MCSession *)session didReceiveCertificate:(NSArray *)certificate fromPeer:(MCPeerID *)peerID certificateHandler:(void (^)(BOOL))certificateHandler {
	certificateHandler(YES); // encryption is .none (matches SimHands) — accept.
}

@end

// ===========================================================================
// VisionOSXRInterface members (defined here to keep the MC/ObjC client isolated
// from the main interface translation unit).
// ===========================================================================

void VisionOSXRInterface::simhands_initialize() {
	simhands_enabled = false;
	simhands_client = nullptr;

	OS *os = OS::get_singleton();
	if (os == nullptr || !os->has_environment("GODOT_SIMHANDS")) {
		return;
	}
	const String value = os->get_environment("GODOT_SIMHANDS");
	if (value.is_empty() || value == "0") {
		return;
	}

	GodotSimHandsClient *client = [[GodotSimHandsClient alloc] initWithService:@"Bonjour"];
	[client start];
	simhands_client = (__bridge_retained void *)client;
	simhands_enabled = true;
	os_log(simhands_log(), "[SimHands] ENABLED via GODOT_SIMHANDS — simulator hand bridge active");
}

void VisionOSXRInterface::simhands_uninitialize() {
	if (simhands_client != nullptr) {
		GodotSimHandsClient *client = (__bridge_transfer GodotSimHandsClient *)simhands_client;
		[client stop];
		simhands_client = nullptr;
		client = nil;
	}
	simhands_enabled = false;
}

// Re-read live calibration from user://simhands_calibration.cfg, throttled (~every 10 frames
// — slider tuning needs a few Hz, not 90). simulator_input.gd writes this file on UDP "K*"
// verbs from the SimControlPanel dev tool. Missing file (panel "Reset") → compiled defaults;
// a parse error (reader caught a mid-write) keeps the last good values and retries next cycle.
static void simhands_reload_calibration() {
	static uint32_t tick = 0;
	if ((tick++ % 10) != 0) {
		return;
	}
	Ref<ConfigFile> cf;
	cf.instantiate();
	const Error err = cf->load("user://simhands_calibration.cfg");
	if (err == ERR_FILE_NOT_FOUND || err == ERR_FILE_CANT_OPEN) {
		SIMHANDS_HAND_KNUCKLE_M = SIMHANDS_DEF_KNUCKLE_M;
		SIMHANDS_PLANE_M = SIMHANDS_DEF_PLANE_M;
		SIMHANDS_DEPTH_M = SIMHANDS_DEF_DEPTH_M;
		SIMHANDS_Y_OFFSET_M = SIMHANDS_DEF_Y_OFFSET_M;
		SIMHANDS_Z_SHAPE_GAIN = SIMHANDS_DEF_Z_SHAPE_GAIN;
		return;
	}
	if (err != OK) {
		return; // mid-write/corrupt — keep last good values, retry next cycle
	}
	SIMHANDS_HAND_KNUCKLE_M = (float)(double)cf->get_value("simhands", "hand_scale", (double)SIMHANDS_DEF_KNUCKLE_M);
	SIMHANDS_PLANE_M = (float)(double)cf->get_value("simhands", "plane", (double)SIMHANDS_DEF_PLANE_M);
	SIMHANDS_DEPTH_M = (float)(double)cf->get_value("simhands", "depth", (double)SIMHANDS_DEF_DEPTH_M);
	SIMHANDS_Y_OFFSET_M = (float)(double)cf->get_value("simhands", "y_offset", (double)SIMHANDS_DEF_Y_OFFSET_M);
	SIMHANDS_Z_SHAPE_GAIN = (float)(double)cf->get_value("simhands", "z_gain", (double)SIMHANDS_DEF_Z_SHAPE_GAIN);
}

void VisionOSXRInterface::apply_simhands_hand_states() {
	if (!simhands_enabled || simhands_client == nullptr) {
		return;
	}

	simhands_reload_calibration();

	GodotSimHandsClient *client = (__bridge GodotSimHandsClient *)simhands_client;
	SimHandsPodFrame frame;
	if (![client copyLatestFrame:&frame]) {
		return; // no data received yet — leave the (empty) ARKit-derived state untouched
	}

	// SimHands' mirror/center mapping: normalized image coords → a head-space
	// direction. The constant 0.5 offsets cancel in (u_i - u_0), so relative
	// joint shape is pure landmark geometry; absolute placement uses u_0.
	auto u_of = [](const SimHandsLandmark &lm) -> Vector3 {
		return Vector3(0.5f - lm.x, 0.5f - lm.y, lm.z - 0.5f);
	};

	bool filled[HAND_INDEX_MAX] = { false, false };

	for (uint32_t h = 0; h < frame.hand_count && h < 2; h++) {
		const SimHandsPodHand &pod = frame.hands[h];
		if (!pod.present) {
			continue;
		}
		const int side = pod.side;
		if (side < 0 || side >= HAND_INDEX_MAX || filled[side]) {
			continue;
		}

		const Vector3 u0 = u_of(pod.lm[0]); // wrist
		const Vector3 u9 = u_of(pod.lm[9]); // middle-finger knuckle
		const float ref = Vector3(u9.x - u0.x, u9.y - u0.y, 0.0f).length(); // xy span (stable; z is noisy)
		if (ref < 1e-4f) {
			continue; // degenerate hand — skip
		}
		const float shape_scale = SIMHANDS_HAND_KNUCKLE_M / ref;

		const Vector3 wrist_place(
				u0.x * SIMHANDS_PLANE_M,
				u0.y * SIMHANDS_PLANE_M + SIMHANDS_Y_OFFSET_M,
				-SIMHANDS_DEPTH_M);

		reset_hand_state((HandIndex)side);
		HandInteractionState &state = hand_interaction_states[side];
		state.tracked = true;
		state.has_joint_data = true;

		// Pass 1: metric joint positions (MediaPipe-indexed).
		Vector3 jpos[21];
		for (int li = 0; li < 21; li++) {
			Vector3 rel = (u_of(pod.lm[li]) - u0) * shape_scale;
			rel.z *= SIMHANDS_Z_SHAPE_GAIN;
			jpos[li] = wrist_place + rel;
		}

		// Palm normal (roll reference for every joint basis): wrist→index-knuckle × wrist→pinky-knuckle.
		Vector3 palm_normal = (jpos[5] - jpos[0]).cross(jpos[17] - jpos[0]);
		if (palm_normal.length() < 1e-5f) {
			palm_normal = Vector3(0.0f, 0.0f, 1.0f);
		}
		palm_normal = palm_normal.normalized();

		// Pass 2: write each joint with position + synthesized orientation (+Y toward its child).
		for (int li = 0; li < 21; li++) {
			const Vector3 distal = jpos[SIMHANDS_DIR_TO[li]] - jpos[SIMHANDS_DIR_FROM[li]];
			HandJointState &js = state.joints[SIMHANDS_JOINT_MAP[li]];
			js.tracked = true;
			js.transform = Transform3D(simhands_basis_from_dir(distal, palm_normal), jpos[li]);
			js.radius = 0.01f;
		}

		// Palm = midpoint(wrist, middle knuckle), oriented up the hand; drives the "default" pose.
		HandJointState &palm = state.joints[XRHandTracker::HAND_JOINT_PALM];
		palm.tracked = true;
		palm.transform = Transform3D(simhands_basis_from_dir(jpos[9] - jpos[0], palm_normal), (jpos[0] + jpos[9]) * 0.5f);
		palm.radius = 0.01f;
		state.palm_transform = palm.transform;
		state.default_transform = palm.transform;
		state.aim_transform = palm.transform;
		state.grip_transform = state.joints[XRHandTracker::HAND_JOINT_WRIST].transform;

		// Pinch (thumb tip ↔ index tip) → controller-tracker trigger proxy.
		const Vector3 thumb = state.joints[XRHandTracker::HAND_JOINT_THUMB_TIP].transform.origin;
		const Vector3 index = state.joints[XRHandTracker::HAND_JOINT_INDEX_FINGER_TIP].transform.origin;
		const float pinch_dist = thumb.distance_to(index);
		state.pinch_value = CLAMP((SIMHANDS_PINCH_ANALOG_MAX_M - pinch_dist) / (SIMHANDS_PINCH_ANALOG_MAX_M - SIMHANDS_PINCH_PRESS_M), 0.0f, 1.0f);
		state.pinch_click = pinch_dist < SIMHANDS_PINCH_PRESS_M;
		state.grasp_value = state.pinch_click ? 1.0f : state.pinch_value;
		state.grasp_click = state.pinch_click;

		filled[side] = true;
	}

	// Any side without SimHands data this frame → ensure it reads untracked.
	for (int i = 0; i < HAND_INDEX_MAX; i++) {
		if (!filled[i]) {
			reset_hand_state((HandIndex)i);
		}
	}

	// Throttled diagnostic (~every 60 frames). print_line does not surface in
	// the sim; os_log does — `log stream --predicate 'subsystem ==
	// "com.agilelens.godotvisionpilot"'`.
	static uint32_t log_counter = 0;
	if ((log_counter++ % 60) == 0) {
		for (int i = 0; i < HAND_INDEX_MAX; i++) {
			const HandInteractionState &s = hand_interaction_states[i];
			if (!s.tracked) {
				continue;
			}
			const Vector3 w = s.joints[XRHandTracker::HAND_JOINT_WRIST].transform.origin;
			const float d = s.joints[XRHandTracker::HAND_JOINT_THUMB_TIP].transform.origin.distance_to(
					s.joints[XRHandTracker::HAND_JOINT_INDEX_FINGER_TIP].transform.origin);
			os_log(simhands_log(), "[SimHands] %{public}s tracked wrist=(%.2f, %.2f, %.2f) thumb-index=%.3fm pinch=%d",
					i == HAND_INDEX_LEFT ? "left" : "right", w.x, w.y, w.z, d, s.pinch_click ? 1 : 0);
		}
	}
}

#else // !TARGET_OS_SIMULATOR — device: complete no-ops (ARKit hands are real there).

void VisionOSXRInterface::simhands_initialize() {}
void VisionOSXRInterface::simhands_uninitialize() {}
void VisionOSXRInterface::apply_simhands_hand_states() {}

#endif // TARGET_OS_SIMULATOR

#endif // VISIONOS_ENABLED
