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
    (occluder_at ?o ?b)      ; Static mirror of obj_at_boxel for occluders only
                             ; (obj_at_boxel is fluent — can't appear in stream domains)
    
    ;; --- Know-If fluent (do we know the value?) ---
    (obj_at_boxel_KIF ?o ?b) ; We know whether ?o is at ?b (true or false)
    
    ;; --- Robot state ---
    (at_config ?q)
    (handempty)
    (holding ?o)
    (obj_pose_known ?o)
    
    ;; --- Stream certified facts ---
    (push_solution ?obj ?b_from ?b_to ?q_start ?q_end ?traj) ; Valid push plan
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
  ;; PUSH: Move an object from one boxel to another
  ;; =========================================================================
  ;; A spatial capability: the robot contacts the object and pushes it.
  ;; Effects are purely spatial — object position and robot config update.
  ;; View state (blocks_view, view_clear) is re-derived automatically.
  (:action push
    :parameters (?obj ?b_from ?b_to ?q_start ?q_end ?traj)
    :precondition (and
      (Boxel ?obj)
      (Boxel ?b_from)
      (Boxel ?b_to)
      (Config ?q_start)
      (Config ?q_end)
      (Trajectory ?traj)
      (is_object ?obj)
      (obj_at_boxel ?obj ?b_from)
      (at_config ?q_start)
      (push_solution ?obj ?b_from ?b_to ?q_start ?q_end ?traj)
      (handempty)
    )
    :effect (and
      (obj_at_boxel ?obj ?b_to)
      (not (obj_at_boxel ?obj ?b_from))
      (at_config ?q_end)
      (not (at_config ?q_start))
    )
  )
  
  ;; =========================================================================
  ;; SENSE: Observe a region to check for an object
  ;; =========================================================================
  ;; Requires clear line of sight to the region.
  ;; Uses the fixed scene camera — no robot positioning needed.
  ;; OPTIMISTIC: Assumes object will be found (replanning handles failures)
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
