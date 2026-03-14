;; =============================================================================
;; Semantic Boxel TAMP Domain - PDDLStream Compatible (Untyped)
;; =============================================================================
;; 
;; Models the robot's CAPABILITIES (push, sense, move, pick, place) rather
;; than any specific scenario. Scenario-specific spatial relationships
;; (e.g., which objects block which regions) are expressed as problem-level
;; init facts using generic predicates like blocks_view_at.
;;
;; Uses derived predicates for visibility: blocks_view and view_clear are
;; computed automatically from object positions — actions only need to
;; update spatial state (obj_at_boxel), not view predicates.
;;
;; Uses Know-If fluents for partial observability.
;; Object types are encoded as predicates: (Boxel ?x), (Obj ?x), etc.

(define (domain boxel-tamp)
  (:requirements :strips :equality :derived-predicates)
  
  (:predicates
    ;; --- Type predicates ---
    (Boxel ?x)
    (Obj ?x)
    (Config ?x)
    (Trajectory ?x)
    (Grasp ?x)
    
    ;; --- Boxel classification ---
    (is_shadow ?b)           ; Region not directly visible from the camera
    (is_object ?b)           ; Physical object (can be pushed, picked)
    (is_free_space ?b)       ; Known empty space
    
    ;; --- Visibility geometry (static) ---
    (blocks_view_at ?obj ?b ?region) ; When ?obj is at ?b, it blocks view to ?region
    
    ;; --- Visibility state (derived — DO NOT use in action effects) ---
    (blocks_view ?obj ?region) ; ?obj currently blocks view to ?region
    (view_blocked ?region)     ; Some object blocks view to ?region
    (view_clear ?region)       ; No object blocks view to ?region
    
    ;; --- Ground truth (actual world state) ---
    (obj_at_boxel ?o ?b)     ; Object ?o is physically at boxel ?b
    
    ;; --- Know-If fluent (do we know the value?) ---
    (obj_at_boxel_KIF ?o ?b) ; We know whether ?o is at ?b (true or false)
    
    ;; --- Robot state ---
    (at_config ?q)
    (handempty)
    (holding ?o)
    (obj_pose_known ?o)
    
    ;; --- Stream certified facts ---
    (valid_grasp ?o ?g)           ; Grasp ?g valid for object ?o
    (motion ?q1 ?q2 ?t)           ; Trajectory ?t from ?q1 to ?q2
    (kin_solution ?o ?b ?g ?q)    ; Config ?q for picking ?o from ?b with ?g
    (config_for_boxel ?q ?b)      ; Config ?q targets boxel ?b (EE inside ?b)
  )
  
  ;; =========================================================================
  ;; DERIVED PREDICATES: Visibility from object positions
  ;; =========================================================================
  ;; blocks_view_at is a static geometric fact in the init state.
  ;; blocks_view is derived: true when an object is currently at a position
  ;; that geometrically blocks a view corridor.
  ;; view_clear is derived via stratified negation: true when no object
  ;; blocks the view. Push never mentions views — it moves objects, and
  ;; the planner re-derives visibility automatically.
  
  (:derived (blocks_view ?obj ?region)
    (exists (?b)
      (and (obj_at_boxel ?obj ?b)
           (blocks_view_at ?obj ?b ?region))))
  
  (:derived (view_blocked ?region)
    (exists (?obj)
      (blocks_view ?obj ?region)))
  
  (:derived (view_clear ?region)
    (and (Boxel ?region)
         (is_shadow ?region)
         (not (view_blocked ?region))))
  
  ;; =========================================================================
  ;; SENSE: Observe a region to check for an object
  ;; =========================================================================
  ;; Requires clear line of sight to the region.
  ;; Uses the fixed scene camera — no robot positioning needed.
  ;;
  ;; DESIGN DECISION — Optimistic sensing with reactive replanning (#61):
  ;;
  ;; The proposal (Section 4.4.2) defines sense with conditional effects:
  ;;   stream_get_sensing_outcome returns found/not_found, and (when ...)
  ;;   clauses set obj_in_Boxel or obj_not_in_Boxel accordingly.  This
  ;;   would let the planner generate multi-step search plans ("sense A;
  ;;   if empty, sense B") within a single plan.
  ;;
  ;; This implementation uses optimistic single-outcome sensing instead:
  ;;   the effect unconditionally assumes the target is found.  When
  ;;   execution reveals it is NOT there, the Python execution loop
  ;;   breaks out, updates the belief state (marks shadow as empty),
  ;;   and replans with updated knowledge.
  ;;
  ;; Justification for this deviation:
  ;;   (a) PDDLStream + FastDownward do not support contingent planning.
  ;;       Conditional effects in PDDL require deterministic outcomes;
  ;;       branching on observation results requires a contingent planner
  ;;       (e.g. POND, CLG), which is outside PDDLStream's architecture.
  ;;   (b) Optimistic planning with replanning on failure is a standard
  ;;       pattern in TAMP under partial observability (Garrett et al.,
  ;;       2020; Kaelbling & Lozano-Pérez, 2013).  PDDLStream's own
  ;;       adaptive algorithm is built around optimistic assumptions.
  ;;   (c) For tabletop scenarios with N shadows, the reactive approach
  ;;       converges in at most N replan cycles (one per empty shadow).
  ;;       The execution loop bounds this: max_replans = 4*N + 1.
  ;;   (d) Belief state propagates correctly across replans: known_empty
  ;;       shadows carry over, so each replan searches strictly fewer
  ;;       candidates.  This is functionally equivalent to the proposal's
  ;;       conditional plan, executed sequentially.
  ;;
  ;; Limitation: the planner cannot reason about search ORDER — it picks
  ;; whichever shadow FastDownward expands first.  A conditional planner
  ;; could optimize search order (e.g. most-likely-first).  For the
  ;; current uniform-prior scenario this does not affect completeness.
  ;;
  ;; See CODEBASE_AUDIT #61, PA-5, PF-1.
  (:action sense
    :parameters (?o ?region)
    :precondition (and
      (Obj ?o)
      (Boxel ?region)
      (view_clear ?region)
      (not (obj_at_boxel_KIF ?o ?region))  ; Only sense if unknown
    )
    :effect (and
      (obj_at_boxel_KIF ?o ?region)        ; Now we know
      (obj_at_boxel ?o ?region)            ; OPTIMISTIC: assume found
      (obj_pose_known ?o)
    )
  )
  
  ;; =========================================================================
  ;; MOVE: Move robot from one configuration to another
  ;; =========================================================================
  ;; ?b is the destination boxel — the stream that produced ?q2 certifies
  ;; that the end-effector at ?q2 is within boxel ?b.
  (:action move
    :parameters (?q1 ?q2 ?b ?t)
    :precondition (and
      (Config ?q1)
      (Config ?q2)
      (Boxel ?b)
      (Trajectory ?t)
      (at_config ?q1)
      (config_for_boxel ?q2 ?b)
      (motion ?q1 ?q2 ?t)
    )
    :effect (and
      (at_config ?q2)
      (not (at_config ?q1))
    )
  )
  
  ;; =========================================================================
  ;; PICK: Pick up an object from a boxel
  ;; =========================================================================
  ;; Must KNOW object is there (KIF=true AND at=true)
  ;;
  ;; NOTE (accepted simplification): pick does not update at_config.
  ;; Physically, execution moves through approach → contact → lift waypoints
  ;; and ends at the lift config, not ?q.  The PDDL state still says
  ;; (at_config ?q) after pick.  This is safe because:
  ;;   (a) The reactive replanning loop re-initializes at_config with the
  ;;       robot's actual joint state before every new plan.
  ;;   (b) Plans rarely chain pick → move without an intervening replan.
  ;; See CODEBASE_AUDIT #62 and PF-2.
  (:action pick
    :parameters (?o ?b ?g ?q)
    :precondition (and
      (Obj ?o)
      (Boxel ?b)
      (Grasp ?g)
      (Config ?q)
      (handempty)
      (at_config ?q)
      (obj_at_boxel_KIF ?o ?b)        ; Must know
      (obj_at_boxel ?o ?b)            ; Must be there
      (kin_solution ?o ?b ?g ?q)
    )
    :effect (and
      (holding ?o)
      (not (handempty))
      (not (obj_at_boxel ?o ?b))
    )
  )
  
  ;; =========================================================================
  ;; PLACE: Place an object in a boxel
  ;; =========================================================================
  ;; Destination must be free space
  ;;
  ;; NOTE (accepted simplification): place does not update at_config.
  ;; Same reasoning as pick — see above and CODEBASE_AUDIT #62 / PF-3.
  (:action place
    :parameters (?o ?b ?g ?q)
    :precondition (and
      (Obj ?o)
      (Boxel ?b)
      (Grasp ?g)
      (Config ?q)
      (holding ?o)
      (at_config ?q)
      (is_free_space ?b)
      (kin_solution ?o ?b ?g ?q)
    )
    :effect (and
      (handempty)
      (obj_at_boxel ?o ?b)
      (obj_at_boxel_KIF ?o ?b)
      (not (holding ?o))
    )
  )
)
